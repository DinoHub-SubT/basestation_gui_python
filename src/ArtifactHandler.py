#!/usr/bin/env python
"""
Functions to handle the artifacts. From detections, to creating new ones, to
keeping track of existing artifacts to submitting artifacts to DARPA
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebastion or Matt).
"""
import rospy
import rospkg
import copy
import cv2
import time
import robots
import json
import base64
import pickle
import os
import re
import threading

import basestation_msgs.msg as bsm

from cv_bridge import CvBridge
from std_msgs.msg import String, UInt8, Bool
from sensor_msgs.msg import Image
from basestation_gui_python.msg import (
    Artifact,
    GuiMessage,
    ArtifactSubmissionReply,
    ArtifactDisplayImage,
    ArtifactUpdate,
    DarpaStatus,
    RetreiveArtifactImage,
    ArtifactVisualizerUpdate,
)
from base_py import BaseNode


class ArtifactHandler(BaseNode):
    """
    Class that keeps track of artifacts and contains utility functions
    to make new ones, delete artifacts, etc.
    """

    def __init__(self):
        super(ArtifactHandler, self).__init__("ArtifactHandler")

    def initialize(self):
        # dictionary of artifacts, indexed by unique_id
        self.all_artifacts = {}
        # dictionary of artifacts currently in the queue
        self.queued_artifacts = {}
        # artifact being displayed in the manipulator plugin
        self.displayed_artifact_id = None
        # the index of the image to display for the artifact focused on
        self.img_ind_displayed = 0
        # artifact image index in the set of images to be displayed
        self.img_displayed = 0
        # dictionary ofartifacts deleted from queue but we may want to keep aroun

        # (future gui development needed to actually use this info)
        self.archived_artifacts = {}
        self.submitted_artifacts = {}

        config = robots.Config()
        # categories from robot are 1-based. so element 0 is unknown
        self.artifact_categories = ["Unknown"]
        for category in config.darpa.artifact_categories:
            self.artifact_categories.append(category)

        self.br = CvBridge()  # bridge from opencv to ros image messages

        # used to submit stuff with information on the gui (and not necessarily in the artifact object)
        self.displayed_pose = [None, None, None]
        self.displayed_category = None

        # variables to hold darpa info in this handler
        self.time_elapsed, self.score, self.remaining_reports = -1, -1, -1

        # Load existing received/submitted artifacts from disk.
        def loadArtifact(where, what, store):
            for p in os.listdir(where):
                p = os.path.join(where, p)
                if os.path.isfile(p) and re.match(r".*\.json$", p):
                    try:
                        with open(p, "r") as f:
                            d = json.load(f)
                            g = GuiArtifact.from_dict(d)
                            self.all_artifacts[g.unique_id] = g
                            store[g.unique_id] = g
                    except Exception as e:
                        m = "[Artifact Handler] Failed to load %s artifact in %s: %s"
                        rospy.logerr(m, what, p, e)

        loadArtifact(self.receivedDirectory(), "received", self.queued_artifacts)
        loadArtifact(self.submittedDirectory(), "submitted", self.submitted_artifacts)

        # subscriber and publishers
        self.message_pub = rospy.Publisher(
            "/gui/message_print", GuiMessage, queue_size=10
        )  # for printing messages
        self.to_queue_pub = rospy.Publisher(
            "/gui/artifact_to_queue", Artifact, queue_size=10
        )  # sending artifacts to the queue
        self.submission_reply_pub = rospy.Publisher(
            "/gui/submission_reply", ArtifactSubmissionReply, queue_size=10
        )  # printing darpa responses
        self.img_display_pub = rospy.Publisher(
            "/gui/img_to_display", ArtifactDisplayImage, queue_size=10
        )  # for sending which image to display to VisualizerPlugin
        self.update_artifact_in_queue_pub = rospy.Publisher(
            "/gui/update_artifact_in_queue", ArtifactUpdate, queue_size=10
        )  # to change artifact info in the queue
        self.remove_artifact_from_queue_pub = rospy.Publisher(
            "/gui/remove_artifact_from_queue", String, queue_size=10
        )  # remove an artifatc from the queue
        self.update_label_pub = rospy.Publisher(
            "/gui/update_art_label", ArtifactVisualizerUpdate, queue_size=10
        )  # send an update to the Visualizer plugin
        self.clear_displayed_img_pub = rospy.Publisher(
            "/gui/clear_img", Bool, queue_size=10
        )  # clear the image being displayed in the Visulizer plugin
        self.submit_tell_queue_pub = rospy.Publisher(
            "/gui/submit_tell_queue", String, queue_size=10
        )  # tell the queue we submitted an artifact

        self.subscriptions = []

        def sub(topic, what, callback):
            s = rospy.Subscriber(topic, what, callback)
            self.subscriptions.append(s)

        # if we generated the artifact manually from the queue
        sub(
            "/gui/generate_new_artifact_manual",
            Artifact,
            self.generateNewArtifactManually,
        )
        # if we selected an artifact from the queue
        sub("/gui/focus_on_artifact", String, self.setDisplayedArtifact)
        # if we deleted an artifact from the queue
        sub("/gui/delete_artifact", String, self.deleteArtifact)
        # if we duplicated an artifact from the queue
        sub("/gui/duplicate_artifact", String, self.duplicateArtifact)
        # if we want to submit an artifact from the queue
        sub("/gui/submit_artifact", ArtifactSubmissionReply, self.submitArtifact)
        # to handle button presses iterating over artifact images
        sub("/gui/change_disp_img", RetreiveArtifactImage, self.getArtifactImage)
        # for updates from the manipulation panel
        sub("/gui/update_artifact_info", ArtifactUpdate, self.updateArtifactInfoFromGui)
        # if we received information from DARPA
        sub("/gui/darpa_status", DarpaStatus, self.updateDarpaInfo)

        def onDetect(robot_uuid):
            def callback(msg):
                msg_id = GuiArtifact.message_id_from_msg(msg, robot_uuid)
                if msg_id in self.queued_artifacts.keys():
                    # we're removing an artifact not adding one
                    if msg.artifact_type == bsm.WifiDetection.ARTIFACT_REMOVE:
                        self.archiveArtifact(String(msg_id))
                        # remove the artifact from the queue
                        remove_msg = String()
                        remove_msg.data = msg_id
                        self.remove_artifact_from_queue_pub.publish(remove_msg)
                    else:  # this is an update for an artifact we already have
                        self.updateWifiDetection(msg, msg_id)
                # this artifact has is a completely new artifact detection
                elif msg_id not in self.all_artifacts.keys():
                    self.generateNewArtifactWifi(msg, robot_uuid)

            return callback

        for r in config.robots:
            detect = "/{0}/{1}".format(r.topic_prefix, r.topics.get("wifi_detection"))
            sub(detect, bsm.WifiDetection, onDetect(r.uuid))

        # We would like to display existing persisted artifacts immediately after the
        # handler is loaded; however, there is a ROS/rqt bootstrap delay that appears to
        # get in the way.  An attempt was made to have various artifact plugins send a
        # 'ready' message at the end of their __init__ call but the message was never
        # received despite it reporting that its initialized had been completed.  With
        # that being said, we can see this ugly hack here of waiting for two seconds
        # before sending all known queued and submitted artifacts.
        threading.Timer(2, self.broadcastArtifacts).start()
        return True

    ##############################################################################
    # Functions to support converting betwen ros messages ansd guiartifact messages
    ##############################################################################

    def guiArtifactToRos(self, artifact):
        """
        Converts an GuiArtifact to a ros message

        artifact is a guiArtifact object
        """
        msg = Artifact()
        msg.category = artifact.category
        msg.curr_pose.position.x = artifact.pose[0]
        msg.curr_pose.position.y = artifact.pose[1]
        msg.curr_pose.position.z = artifact.pose[2]
        msg.orig_pose.position.x = artifact.orig_pose[0]
        msg.orig_pose.position.y = artifact.orig_pose[1]
        msg.orig_pose.position.z = artifact.orig_pose[2]
        msg.robot_uuid = artifact.robot_uuid
        msg.source_robot_id = artifact.source_robot_id
        msg.artifact_report_id = artifact.artifact_report_id
        msg.time_from_robot = artifact.time_from_robot
        msg.time_to_darpa = artifact.time_to_darpa
        msg.unread = artifact.unread
        msg.priority = ""
        msg.darpa_response = artifact.darpa_response
        msg.img_stamps = artifact.img_stamps
        msg.original_timestamp = artifact.original_timestamp
        msg.unique_id = artifact.unique_id
        # convert the mages from numpy to ros
        ros_imgs = []
        for img in artifact.imgs:
            ros_img = self.br.cv2_to_imgmsg(img)
            ros_imgs.append(ros_img)
        msg.imgs = ros_imgs
        return msg

    def rosArtifactToGuiArtifact(self, msg):
        """
        Converts a Ros artifact to a GuiArtifact

        msg is a custom Artifact ros message
        """
        if msg.unique_id in self.all_artifacts.keys():  # we already have this artifact
            artifact = self.all_artifacts[msg.unique_key]
        else:  # we need to generate a new artifact
            # We're manually generating an artifact, make a new artifact id for it.
            if msg.artifact_report_id == -1:
                # go find the smallest id, and increment it by 1 to generate new id
                min_negative_id = 0
                for artifact_key in self.all_artifacts.keys():
                    artifact = self.all_artifacts[artifact_key]
                    if artifact.artifact_report_id < min_negative_id:
                        min_negative_id = artifact.artifact_report_id
                artifact_report_id = min_negative_id - 1
            else:
                artifact_report_id = msg.artifact_report_id

            # generate new artifact
            artifact = GuiArtifact(
                original_timestamp=float(msg.original_timestamp),
                category=msg.category,
                pose=[
                    msg.orig_pose.position.x,
                    msg.orig_pose.position.y,
                    msg.orig_pose.position.z,
                ],
                source_robot_id=msg.source_robot_id,
                artifact_report_id=artifact_report_id,
                imgs=msg.imgs,
                img_stamps=msg.img_stamps,
                time_from_robot=self.time_elapsed,
                robot_uuid=msg.robot_uuid,
            )
        return artifact

    def updateArtifactInfoFromGui(self, msg):
        """
        Update an artifact's info based on some gui interaction

        msg is a custom ArtifactInfo message
        """
        artifact = self.all_artifacts[msg.unique_id]
        categories = [
            ArtifactUpdate.PROPERTY_CATEGORY,
            ArtifactUpdate.PROPERTY_POSE_X,
            ArtifactUpdate.PROPERTY_POSE_Y,
            ArtifactUpdate.PROPERTY_POSE_Z,
        ]

        if msg.update_type not in categories:
            data = "Received unknown artifact property, {0}.  Not updated."
            update_msg = GuiMessage()
            update_msg.data = data.format(msg.updateType)
            update_msg.color = update_msg.COLOR_RED
            self.message_pub.publish(update_msg)
        else:
            artifact.category = msg.category
            artifact.pose = [
                msg.curr_pose.position.x,
                msg.curr_pose.position.y,
                msg.curr_pose.position.z,
            ]
            self.displayed_pose = artifact.pose
            self.displayed_category = artifact.category
            self.persistArtifact(artifact, self.receivedDirectory())
            self.update_artifact_in_queue_pub.publish(msg)
            idx = -1
            if len(artifact.imgs) > 0:
                idx = self.img_ind_displayed
            self.publishImgToDisplay(idx)

    ##############################################################################
    # Functions to support generating artifacts from wifi detections,
    # radio detections, or manually
    ##############################################################################

    def timestampsOnImgs(self, msg_imgs):
        """
        Function to put timestamps on images.

        msg_imgs is an array of Images
        """
        imgs = []
        img_stamps = []
        for img in msg_imgs:
            cv_image = self.br.imgmsg_to_cv2(img)
            cv2.putText(
                cv_image,
                "Timestamp: %f" % img.header.stamp.to_sec(),
                (5, 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )
            imgs.append(cv_image)
            img_stamps.append(img.header.stamp)
        return imgs, img_stamps

    def updateWifiDetection(self, msg, msg_unique_id):
        """
        Updated information for an artifact we already have has
        been sent from the robot via Wifi

        msg is a WifiDetection message containing artifact info
        msg_unique_id is the unique id of the artifact being sent
        """
        artifact = self.all_artifacts[msg_unique_id]

        # now we need to check was information is passed in the message
        # if a field has something other than the default value sent back, we need to update
        # the artifact
        updated = False  # if it has been updated, send a notification to the queue

        if msg.artifact_type != 0:
            artifact.category = self.artifact_categories[msg.artifact_type]
            updated = True
            # publish to change the value in the queue. we only do this for changes in the type
            # because that's the only relevant property that the queue displays
            queue_msg = ArtifactUpdate()
            queue_msg.unique_id = artifact.unique_id
            queue_msg.update_type = ArtifactUpdate.PROPERTY_CATEGORY
            queue_msg.category = artifact.category
            self.update_artifact_in_queue_pub.publish(queue_msg)

        if (msg.artifact_x != 0) or (msg.artifact_y != 0) or (msg.artifact_z != 0):
            artifact.pose = [msg.artifact_x, msg.artifact_y, msg.artifact_z]
            updated = True

        # over-write all of the images!! probably should be fixed.
        if len(msg.imgs) > 0:
            artifact.imgs, artifact.img_stamps = self.timestampsOnImgs(msg.imgs)
            updated = True

        if updated and msg_unique_id == self.displayed_artifact_id:
            # if we updated the artifact being displayed,
            # send a message to the image visualization panel to re-select the artifact from the queue
            update_msg = ArtifactVisualizerUpdate()
            update_msg.data = ArtifactVisualizerUpdate.UPDATE
            self.update_label_pub.publish(update_msg)

    def generateNewArtifactWifi(self, msg, robot_uuid):
        """
        Generate a new artifact which has been detected from the robot
        and trasmitted via WiFi.

        msg is a WifiDetection message containing info about the artifact detected
        """
        # msg.category is an int using a agreed-upon convention
        artifact_category = self.artifact_categories[msg.artifact_type]
        # put timestamps on the images and extract timestamp info
        imgs, img_stamps = self.timestampsOnImgs(msg.imgs)
        artifact = GuiArtifact(
            robot_uuid=robot_uuid,
            original_timestamp=msg.artifact_stamp.secs,
            category=artifact_category,
            pose=[msg.artifact_x, msg.artifact_y, msg.artifact_z],
            source_robot_id=msg.artifact_robot_id,
            artifact_report_id=msg.artifact_report_id,
            imgs=imgs,
            img_stamps=img_stamps,
            time_from_robot=self.robotTimeToDarpaTime(msg.artifact_stamp.to_sec()),
        )
        self.bookeepAndPublishNewArtifact(artifact)

    def generateNewArtifactManually(self, msg):
        """
        Generate a new artifact from a button press for manually-adding one.
        Input message is custom ROS artifact message

        msg is custom Artifact message containign some default info about artifacts
        """
        # fill in some of the info for the message being published to add to queue
        artifact = self.rosArtifactToGuiArtifact(msg)
        self.bookeepAndPublishNewArtifact(artifact)

    def duplicateArtifact(self, msg):
        """
        Take in the unique id of an artifact to duplicate and
        duplicates it

        msg is a string that is the artifact unique_id
        """
        artifact_to_dup = self.all_artifacts[msg.data]
        if artifact_to_dup != None:
            # find a unique negative id. manually generated artifacts have a negative id
            negative_id_list = []
            for art in self.all_artifacts.keys():
                if self.all_artifacts[art].artifact_report_id < 0:
                    negative_id_list.append(self.all_artifacts[art].artifact_report_id)

            artifact_id = (len(negative_id_list) + 1) * -1

            # make the robot_id negative as well.
            art_source_id = (
                artifact_to_dup.source_robot_id + 1
            ) * -1  #'+1' because one of the robot ids is 0, which won't go negative
            # with '*-1'

            # generate the artifact object
            artifact = GuiArtifact(
                copy.deepcopy(artifact_to_dup.original_timestamp),
                copy.deepcopy(artifact_to_dup.category),
                copy.deepcopy(artifact_to_dup.pose),
                art_source_id,
                artifact_id,
                copy.deepcopy(artifact_to_dup.imgs),
                copy.deepcopy(artifact_to_dup.img_stamps),
                time_from_robot=copy.deepcopy(artifact_to_dup.time_from_robot),
                robot_uuid=copy.deepcopy(artifact_to_dup.robot_uuid),
            )

            # add the artifact to the list of queued objects and to the all_artifacts list
            self.queued_artifacts[artifact.unique_id] = artifact
            self.all_artifacts[artifact.unique_id] = artifact

            # publish this message to be visualized by plugins
            # necessary step to fill in some defaults (i.e. category)
            # to be used by other parts of the gui
            ros_msg = self.guiArtifactToRos(artifact)

            # add the artifact to the queue
            self.to_queue_pub.publish(ros_msg)

    ##############################################################################
    # Functions to support DARPA artifact proposals
    ##############################################################################

    def submitArtifact(self, reply):
        """
        Process an artifact submission reply from DARPA by forwarding the reply and
        update internal bookkeeping.

        Reply is of message type ArtifactSubmissionReply.
        """
        uid = reply.unique_id
        artifact = self.all_artifacts[uid]
        if artifact == None:
            m = "Artifact with unique ID {0} not found. Artifact not submitted."
            g = GuiMessage()
            g.data = m.format(str(msg.data))
            g.color = g.COLOR_RED
            self.message_pub.publish(g)
            rospy.logerr("[Artifact Handler] " + g.data)
            return

        artifact.darpa_submit_time = reply.submission_time_raw
        artifact.darpa_artifact_type = reply.artifact_type
        artifact.darpa_score_change = reply.score_change
        artifact.darpa_report_status = reply.report_status
        artifact.darpa_response = reply.http_response
        artifact.darpa_reason = reply.http_reason
        artifact.darpa_x = reply.x
        artifact.darpa_y = reply.y
        artifact.darpa_z = reply.z

        self.submission_reply_pub.publish(reply)

        # Remove the artifact from the book keeping it may have
        # already been removed if the artifact was deleted.
        if uid in self.queued_artifacts.keys():
            self.queued_artifacts.pop(uid)
            self.removePersistedArtifact(artifact, self.receivedDirectory())
            self.clear_displayed_img_pub.publish(Bool(True))

        self.all_artifacts[uid] = artifact
        self.submitted_artifacts[uid] = artifact
        self.persistArtifact(artifact, self.submittedDirectory())

        # Remove the artifact update message from the image visualizer plugin
        # if it is still visible.
        viz = ArtifactVisualizerUpdate()
        viz.data = ArtifactVisualizerUpdate.HIDE
        self.update_label_pub.publish(viz)

        # Nothing is being displayed, so reset bookeeping.
        self.displayed_pose = [None, None, None]
        self.displayed_category = None
        self.displayed_artifact_id = None

    ##############################################################################
    # Functions to support artifact manipulation/visualization
    ##############################################################################

    def setDisplayedArtifact(self, msg):
        """
        Set the artifact we're going to be visualizing/manipulating/etc.
        Incoming message is a string of the unique id of the artifact we selected

        msg is a string containing the artifact id
        """
        self.displayed_artifact_id = msg.data
        self.img_ind_displayed = 0  # reset the image index we're displaying
        self.getArtifactImage(UInt8(2))  # display the first image in the detection

    def getArtifactImage(self, msg):
        """
        Get another artifact image to show.

        msg.data defines whethere we go forward in the set or backward
        """
        if self.displayed_artifact_id == None:
            g = GuiMessage()
            g.data = "No artifact selected.  No image to display."
            g.color = g.COLOR_ORANGE
            self.message_pub.publish(g)
            return

        # display a black image because this artifact has no images
        direction = msg.data
        img_count = len(self.all_artifacts[self.displayed_artifact_id].imgs)
        idx = -1

        if img_count == 0:
            idx = -1
        elif direction == RetreiveArtifactImage.DIRECTION_FORWARD:
            idx = (self.img_ind_displayed + 1) % img_count
            self.img_ind_displayed = idx
        elif direction == RetreiveArtifactImage.DIRECTION_BACKWARD:
            idx = (self.img_ind_displayed - 1) % img_count
            self.img_ind_displayed = idx
        elif direction == RetreiveArtifactImage.DISPLAY_FIRST and img_count > 0:
            idx = 0
        else:
            g = GuiMessage()
            g.data = "Invalid direction. Image not changed."
            g.color = g.COLOR_RED
            self.message_pub.publish(g)

        self.publishImgToDisplay(idx)

    def publishImgToDisplay(self, ind):
        """
        Publish an image to be displayed in the artifact visualization panel.

        ind is the index in the artifact's image list of the image to be displayed.
        """
        if self.displayed_artifact_id == None:
            # Nothing selected in the table so nothing to display.
            return

        msg = ArtifactDisplayImage()
        art = self.all_artifacts[self.displayed_artifact_id]
        if ind == -1:  # this artifact contains no images display the blank black image
            path = rospkg.RosPack().get_path("basestation_gui_python")
            name = "{0}/src/black_img.png".format(path)
            img = cv2.imread(name)
            msg.img = self.br.cv2_to_imgmsg(img)
            msg.image_ind = 0
            msg.num_images = 0
            msg.pose = art.pose
        else:
            img = art.imgs[ind]
            msg.img = self.br.cv2_to_imgmsg(img)
            msg.image_ind = ind
            msg.num_images = len(art.imgs)
            msg.pose = art.pose

        self.img_display_pub.publish(msg)

    ##############################################################################
    # Misc. functions
    ##############################################################################

    def updateDarpaInfo(self, msg):
        """
        We just got a darpa status update. Update our local info accordingly

        msg is a DarpaStatus custom message containing general run info from DARPA
        """
        self.time_elapsed = msg.time_elapsed
        self.score = msg.score
        self.remaining_reports = msg.remaining_reports

    def robotTimeToDarpaTime(self, robot_time):
        """
        Convert robot_time (in float seconds) to be aligned to DARPA time.

        If the DARPA time isn't available, the original time is returned.
        """
        if self.time_elapsed == -1:
            return robot_time
        current_time = rospy.get_time()
        time_offset = current_time - self.time_elapsed
        return robot_time - time_offset

    def receivedDirectory(self):
        p = rospkg.RosPack().get_path("basestation_gui_python")
        return os.path.join(p, "data", "artifacts", "received")

    def submittedDirectory(self):
        p = rospkg.RosPack().get_path("basestation_gui_python")
        return os.path.join(p, "data", "artifacts", "submitted")

    def persistArtifact(self, artifact, directory):
        p = os.path.join(directory, artifact.filename())
        with open(p, "w") as f:
            d = artifact.to_dict()
            json.dump(d, f)

    def removePersistedArtifact(self, artifact, directory):
        p = os.path.join(directory, artifact.filename())
        try:
            os.remove(p)
        except Exception as e:
            m = "[Artifact Handler] Failed to remove artifact file %s: %s"
            rospy.logerr(m, p, e)

    def deleteArtifact(self, msg):
        """
        Used when an artifact should be removed from the queue.

        msg is a string of the unique_id
        """
        # go find the artifact
        artifact = self.all_artifacts[msg.data]
        if artifact != None:
            # default return value is None if key not found
            self.queued_artifacts.pop(artifact.unique_id)
            self.removePersistedArtifact(artifact, self.receivedDirectory())

            g = GuiMessage()
            g.data = "Artifact removed: " + artifact.unique_id
            g.color = g.COLOR_GREEN
            self.message_pub.publish(g)
            # if the artifact has images displayed, we want to send a message syaing its been deleted
            if artifact.unique_id == self.displayed_artifact_id:
                u = ArtifactVisualizerUpdate()
                u.data = ArtifactVisualizerUpdate.DELETE
                self.update_label_pub.publish(u)
        else:
            g = GuiMessage()
            g.data = "[Artifact Handler] No such artifact"
            g.color = g.COLOR_RED
            self.message_pub.publish(g)

    def broadcastArtifacts(self):
        """Publishes all known recieved and submitted artifacts."""
        for uid, art in self.queued_artifacts.iteritems():
            m = self.guiArtifactToRos(art)
            self.to_queue_pub.publish(m)
        for uid, art in self.submitted_artifacts.iteritems():
            r = ArtifactSubmissionReply()
            r.unique_id = art.unique_id
            r.submission_time_raw = art.darpa_submit_time
            r.artifact_type = art.darpa_artifact_type
            r.x = art.darpa_x
            r.y = art.darpa_y
            r.z = art.darpa_z
            r.report_status = art.darpa_report_status
            r.score_change = art.darpa_score_change
            r.http_response = art.darpa_response
            r.http_reason = art.darpa_reason
            self.submission_reply_pub.publish(r)

    def bookeepAndPublishNewArtifact(self, artifact):
        """
        Add the new artifact to the various lists and publish the
        info that we have a new artofact to the various channels

        artifact is a GuiArtifact object
        """
        self.all_artifacts[artifact.unique_id] = artifact
        self.queued_artifacts[artifact.unique_id] = artifact
        self.persistArtifact(artifact, self.receivedDirectory())

        # publish this message to be visualized by plugins
        # necessary step to fill in some defaults (i.e. category)
        # to be used by other parts of the gui
        ros_msg = self.guiArtifactToRos(artifact)
        # add the artifact to the queue
        self.to_queue_pub.publish(ros_msg)

    def execute(self):
        return True

    def shutdown(self):
        for s in self.subscriptions:
            s.unregister()
        rospy.loginfo("[Artifact Handler] shutting down")


class GuiArtifact:
    """Class to handle artifacts as an object in the gui."""

    def __init__(
        self,
        original_timestamp=1,
        category=-1,
        pose="",
        source_robot_id="",
        artifact_report_id="",
        imgs=None,
        img_stamps=None,
        time_from_robot=None,
        robot_uuid=None,
    ):
        self.category = category
        self.pose = pose
        self.orig_pose = copy.deepcopy(pose)
        self.robot_uuid = robot_uuid
        self.source_robot_id = source_robot_id
        self.artifact_report_id = artifact_report_id
        # time the detection has come in from the robot. TODO: change to be something different?
        self.time_from_robot = time_from_robot if time_from_robot is not None else -1
        self.time_to_darpa = -1  # time submitted to darpa
        self.unread = True
        self.darpa_submit_time = 0.0
        self.darpa_artifact_type = ""
        self.darpa_score_change = 0
        self.darpa_report_status = ""
        self.darpa_response = ""
        self.darpa_reason = ""
        self.darpa_x = 0.0
        self.darpa_y = 0.0
        self.darpa_z = 0.0
        self.imgs = imgs if imgs is not None else []
        # time stamps of the images
        self.img_stamps = img_stamps if img_stamps is not None else []
        self.original_timestamp = original_timestamp
        self.unique_id = GuiArtifact.message_id(
            robot_uuid, artifact_report_id, original_timestamp
        )

    def filename(self):
        return self.unique_id.replace("/", "_") + ".json"

    def to_dict(self):
        """
        Encode a GuiArtifact to a python dictionary that is suitable for JSON encoding.
        """
        d = {
            "category": self.category,
            "pose": self.pose,
            "orig_pose": self.orig_pose,
            "robot_uuid": self.robot_uuid,
            "source_robot_id": self.source_robot_id,
            "artifact_report_id": self.artifact_report_id,
            "time_from_robot": self.time_from_robot,
            "time_to_darpa": self.time_to_darpa,
            "unread": self.unread,
            "darpa_submit_time": self.darpa_submit_time,
            "darpa_artifact_type": self.darpa_artifact_type,
            "darpa_score_change": self.darpa_score_change,
            "darpa_report_status": self.darpa_report_status,
            "darpa_repsonse": self.darpa_response,
            "darpa_reason": self.darpa_reason,
            "darpa_x": self.darpa_x,
            "darpa_y": self.darpa_y,
            "darpa_z": self.darpa_z,
            "original_timestamp": self.original_timestamp,
            "unique_id": self.unique_id,
        }
        imgs = []
        for i in self.imgs:
            s = i.dumps()
            b = base64.b64encode(s)
            imgs.append(b)
        d["imgs"] = imgs

        img_stamps = []
        for stamp in self.img_stamps:
            s = stamp.to_sec()
            img_stamps.append(s)
        d["img_stamps"] = img_stamps

        return d

    @staticmethod
    def from_dict(d):
        """
        Decode a python dictionary that was created with GuiArtifact.to_dict.
        """
        imgs = []
        for b64 in d["imgs"]:
            s = base64.b64decode(b64)
            i = pickle.loads(s)
            imgs.append(i)

        img_stamps = []
        for stamp in d["img_stamps"]:
            t = rospy.Time.from_sec(stamp)
            img_stamps.append(t)

        uid = d["robot_uuid"]
        orig_ts = d["original_timestamp"]
        report_id = d["artifact_report_id"]

        # Performing partial construction here in order to modifying the GuiArtifact
        # constructor.  It needs these parameters to not throw an error when
        # constructing the artifacts unique ID despite us overridding the value from the
        # previous serialized value.
        g = GuiArtifact(
            original_timestamp=orig_ts, artifact_report_id=report_id, robot_uuid=uid
        )
        g.imgs = imgs
        g.img_stamps = img_stamps
        g.category = d["category"]
        g.pose = d["pose"]
        g.orig_pose = d["orig_pose"]
        g.source_robot_id = d["source_robot_id"]
        g.time_from_robot = d["time_from_robot"]
        g.time_to_darpa = d["time_to_darpa"]
        g.unread = d["unread"]
        g.darpa_submit_time = d["darpa_submit_time"]
        g.darpa_artifact_type = d["darpa_artifact_type"]
        g.darpa_score_change = d["darpa_score_change"]
        g.darpa_report_status = d["darpa_report_status"]
        g.darpa_response = d["darpa_repsonse"]
        g.darpa_reason = d["darpa_reason"]
        g.darpa_x = d["darpa_x"]
        g.darpa_y = d["darpa_y"]
        g.darpa_z = d["darpa_z"]
        g.unique_id = d["unique_id"]

        return g

    @staticmethod
    def message_id(robot_uuid, report_id, secs):
        return "{0}/{1}/{2}".format(robot_uuid[0:8], report_id, secs)

    @staticmethod
    def message_id_from_msg(msg, robot_uuid):
        report = msg.artifact_report_id
        secs = msg.artifact_stamp.secs
        return GuiArtifact.message_id(robot_uuid, report, secs)


if __name__ == "__main__":
    node = ArtifactHandler()
    node.run()
