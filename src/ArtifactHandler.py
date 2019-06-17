#!/usr/bin/env python
"""
Functions to handle the artifacts. From detections, to creating new ones, to
keeping track of existing artifacts to submitting artifacts to DARPA
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebastion or Matt).
"""
import rospy
import copy
import rospkg
import cv2
import time
import robots

from cv_bridge import CvBridge
from darpa_command_post.TeamClient import TeamClient, ArtifactReport

import basestation_msgs.msg as bsm

from std_msgs.msg import String, UInt8, Bool
from sensor_msgs.msg import Image
from basestation_gui_python.msg import (
    Artifact,
    GuiMessage,
    ArtifactSubmissionReply,
    WifiDetection,
    ArtifactDisplayImage,
    ArtifactUpdate,
    RadioMsg,
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

        self.http_client = TeamClient()  # client to interact with darpa scoring server
        self.br = CvBridge()  # bridge from opencv to ros image messages

        # used to submit stuff with information on the gui (and not necessarily in the artifact object)
        self.displayed_pose = [None, None, None]
        self.displayed_category = None

        # variables to hold darpa info in this handler
        self.time_elapsed, self.score, self.remaining_reports = -1, -1, -1

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
        self.manipulation_info_pub = rospy.Publisher(
            "/gui/refresh_manipulation_info", Artifact, queue_size=10
        )  # for sending info to the manipulation plugin
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
        # if we archived an artifact from the queue
        sub("/gui/archive_artifact", String, self.archiveArtifact)
        # if we duplicated an artifact from the queue
        sub("/gui/duplicate_artifact", String, self.duplicateArtifact)
        # if we want to submit an artifact from the queue
        sub(
            "/gui/submit_artifact_from_queue",
            String,
            self.buildArtifactSubmissionFromQueue,
        )
        # if we want to submit an artifact from the manipulation panel
        sub(
            "/gui/submit_artifact_from_manip_plugin",
            String,
            self.buildArtifactSubmissionFromManipPlugin,
        )
        # to handle button presses iterating over artifact images
        sub("/gui/change_disp_img", UInt8, self.getArtifactImage)
        # for updates from the manipulation panel
        sub("/gui/update_artifact_info", ArtifactUpdate, self.updateArtifactInfoFromGui)
        # if we received information from DARPA
        sub("/gui/darpa_status", DarpaStatus, self.updateDarpaInfo)
        # The next two subscriptions are for the object detection
        # module which does not use the robot topic prefix for it's
        # testing.  Note, that when deployed on the robot it does use
        # the prefix.  We leave these here compatibility and to avoid
        # having change that module.
        sub("/real_artifact_detections", RadioMsg, self.handleRadioDetection)
        sub("/real_artifact_imgs", WifiDetection, self.handleWifiDetection)
        # These two subscriptions are kept around for legacy purposes despite
        # no longer using fake artifact detections.
        sub("/fake_artifact_detections", RadioMsg, self.handleRadioDetection)
        sub("/fake_artifact_imgs", WifiDetection, self.handleWifiDetection)

        def topic(robot, what):
            return "/{0}/{1}".format(robot.topic_prefix, robot.topics.get(what))

        for r in config.robots:
            wifi = topic(r, "artifact_wifi")
            radio = topic(r, "artifact_radio")
            detect = topic(r, "wifi_detection")
            sub(wifi, WifiDetection, self.handleWifiDetection)
            sub(radio, RadioMsg, self.handleRadioDetection)
            sub(detect, bsm.WifiDetection, self.handleWifiDetection)

        return True

    ##############################################################################
    # Functions to handle detections from the robots over wifi or radio
    ##############################################################################

    def handleWifiDetection(self, msg):
        """
	An artifact detection or update has come in over wifi. Generate a
	new artifact and save to proper dictionaries/lists, or update an existing
	artifact.

	msg is a WifiDetection message from the robot
        """
        msg_unique_id = GuiArtifact.message_id_from_msg(msg)
        if msg_unique_id in self.queued_artifacts.keys():
            # we're removing an artifact not adding one
            if msg.artifact_type == WifiDetection.ARTIFACT_REMOVE:
                self.archiveArtifact(String(msg_unique_id))
                # remove the artifact from the queue
                remove_msg = String()
                remove_msg.data = msg_unique_id
                self.remove_artifact_from_queue_pub.publish(remove_msg)
            else:  # this is an update for an artifact we already have
                self.updateWifiDetection(msg, msg_unique_id)
        # this artifact has is a completely new artifact detection
        elif msg_unique_id not in self.all_artifacts.keys():
            self.generateNewArtifactWifi(msg)

    def handleRadioDetection(self, msg):
        """
	An artifact detection or update has come in over the radio. Generate a
	new artifact and save to proper dictionaries/lists, or update an existing
	artifact.

	msg is a custom RaidoMsg message containing info about the artifact detected
        """
        msg_unique_id = GuiArtifact.message_id_from_msg(msg)
        # if it is an artifact report and it is not archived (i.e. still queued)
        if msg.message_type == RadioMsg.MESSAGE_TYPE_ARTIFACT_REPORT:
            if msg_unique_id in self.queued_artifacts.keys():
                # we're removing an artifact not adding one
                if msg.artifact_type == RadioMsg.ARTIFACT_REMOVE:
                    self.archiveArtifact(String(msg_unique_id))
                    # remove the artifact from the queue
                    remove_msg = String()
                    remove_msg.data = msg_unique_id
                    self.remove_artifact_from_queue_pub.publish(remove_msg)
                else:  # this is an update for an artifact we already have
                    self.updateRadioDetection(msg, msg_unique_id)
            # this artifact has is a completely new artifact detection
            elif msg_unique_id not in self.all_artifacts.keys():
                self.generateNewArtifactRadio(msg)
        # The message type is not and artifact report and thus not
        # what we would expect to receive from the robot.
        elif msg.message_type != RadioMsg.MESSAGE_TYPE_ARTIFACT_REPORT:
            update_msg = GuiMessage()
            update_msg.data = (
                "Received radio message with an unexpected message type.  Discarded."
            )
            update_msg.color = update_msg.COLOR_ORANGE
            self.message_pub.publish(update_msg)

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
            update_msg = GuiMessage()
            update_msg.data = (
                "Received update message of unknown type.  Artifact not updated"
            )
            update_msg.color = update_msg.COLOR_RED
            self.message_pub.publish(update_msg)
        else:
            if msg.update_type == ArtifactUpdate.PROPERTY_CATEGORY:

                artifact.category = msg.category
                self.displayed_category = artifact.category
                # change the value in the artifact queue
                self.update_artifact_in_queue_pub.publish(msg)
            elif msg.update_type == ArtifactUpdate.PROPERTY_POSE_X:
                artifact.pose[0] = msg.curr_pose.position.x
                self.displayed_pose[0] = artifact.pose[0]
            elif msg.update_type == ArtifactUpdate.PROPERTY_POSE_Y:
                artifact.pose[1] = msg.curr_pose.position.y
                self.displayed_pose[1] = artifact.pose[1]
            elif msg.update_type == ArtifactUpdate.PROPERTY_POSE_Z:
                artifact.pose[2] = msg.curr_pose.position.z
                self.displayed_pose[2] = artifact.pose[2]

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

    def generateNewArtifactWifi(self, msg):
        """
        Generate a new artifact which has been detected from the robot
        and trasmitted via WiFi.

        msg is a WifiDetection message containing info about the artifact detected
        """
        # decode the type
        artifact_category = self.artifact_categories[
            msg.artifact_type
        ]  # msg.category is an int using a agreed-upon convention

        # put timestamps on the images and extract timestamp info
        imgs, img_stamps = self.timestampsOnImgs(msg.imgs)

        artifact = GuiArtifact(
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

    def updateRadioDetection(self, msg, msg_unique_id):
        """
        Updated information for an artifact we already have has
        been sent from the robot via radio.

        msg is a RadioMsg message containing artifact info
        msg_unique_id is the unique id of the artifact being sent
        """
        artifact = self.all_artifacts[msg_unique_id]

        # Now we need to check was information is passed in the message if a field has
        # something other than the default value sent back, we need to update the
        # artifact.
        if msg.message_type == RadioMsg.MESSAGE_TYPE_ARTIFACT_REPORT:
            updated = False  # if it has been updated, send a notification to the queue
            if msg.artifact_type != 0:
                artifact.category = self.artifact_categories[msg.artifact_type]
                type_updated = True
                # publish to change the value in the queue. we only do this for changes in the type
                # because that's the only relevant property that the queue displays
                queue_msg = ArtifactUpdate()
                queue_msg.unique_id = artifact.unique_id
                queue_msg.update_type = ArtifactUpdate.PROPERTY_CATEGORY
                queue_msg.category = artifact.category
                self.update_artifact_in_queue_pub.publish(queue_msg)

            if (msg.artifact_x != 0) or (msg.artifact_y != 0) or (msg.artifact_z != 0):
                artifact.pose = [msg.artifact_x, msg.artifact_y, msg.artifact_z]
                pose_updated = True

            if updated and msg_unique_id == self.displayed_artifact_id:
                # if we updated the artifact being displayed,
                # send a message to the image visualization panel to re-select the artifact from the queue
                update_msg = ArtifactVisualizerUpdate()
                update_msg.data = ArtifactVisualizerUpdate.UPDATE
                self.update_label_pub.publish(update_msg)

    def generateNewArtifactRadio(self, msg):
        """
        Generate a new artifact which has been detected from the robot
        and trasmitted via radio.

        msg is of type RadioMsg
        """
        artifact_category = self.artifact_categories[msg.artifact_type]
        artifact = GuiArtifact(
            original_timestamp=msg.artifact_stamp.secs,
            category=artifact_category,
            pose=[msg.artifact_x, msg.artifact_y, msg.artifact_z],
            source_robot_id=msg.artifact_robot_id,
            artifact_report_id=msg.artifact_report_id,
            imgs=[],
            img_stamps=[],
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

    def buildArtifactSubmissionFromQueue(self, msg):
        """
        Build an artifact report to DARPA from the queue

        msg is just a string that's the unqiue_id of the artifact to submit
        """
        artifact = self.all_artifacts[msg.data]
        if artifact != None:
            artifact_report = ArtifactReport(
                x=float(artifact.pose[0]),
                y=float(artifact.pose[1]),
                z=float(artifact.pose[2]),
                type=str(artifact.category),
            )
            self.submitArtifact(artifact_report, artifact.unique_id)
        else:  # we could not find the artifact unique_id
            update_msg = GuiMessage()
            update_msg.data = (
                "Artifact with unique id: "
                + str(msg.data)
                + " not found. Artifact not submitted."
            )
            update_msg.color = update_msg.COLOR_RED
            self.message_pub.publish(update_msg)

    def buildArtifactSubmissionFromManipPlugin(self, msg):
        """
        We are submitting an artifact from the manipulation plugin Thus, we will pull
        the information from what is being displayed on the gui itself not what the
        artifact object may contain.

        msg is just a string that's the unqiue_id of the artifact to submit
        """
        artifact = self.all_artifacts[msg.data]
        if artifact != None:
            pose_is_none = (
                self.displayed_pose[0] != None
                and self.displayed_pose[1] != None
                and self.displayed_pose[2] != None
                and self.displayed_category != None
            )
            # we have been keeping track of what's on the gui
            # ensure this info has been filled in
            if pose_is_none:
                artifact_report = ArtifactReport(
                    x=float(self.displayed_pose[0]),
                    y=float(self.displayed_pose[1]),
                    z=float(self.displayed_pose[2]),
                    type=str(self.displayed_category),
                )
                submission_fine = self.submitArtifact(
                    artifact_report, artifact.unique_id
                )
                # remove the artifact from the queue
                if submission_fine == True:
                    remove_msg = String()
                    remove_msg.data = msg.data
                    self.remove_artifact_from_queue_pub.publish(remove_msg)
                    self.submit_tell_queue_pub.publish(String(artifact.unique_id))

    def submitArtifact(self, artifact_report, artifact_unique_id):
        """
        Submit an artifact report to DARPA.

        artifact_report is a custom dataype just containing the relevat info for an artifact submission
        i.e. position and category

        artifact_unique_id is a string that is the artifact unique id. used for bookeeping after a successful submission
        """
        results = self.http_client.send_artifact_report(artifact_report)
        if results != []:  # we actually get something back
            artifact_report_reply, http_status, http_reason = results
            proposal_return = [
                artifact_report_reply["run_clock"],
                artifact_report_reply["type"],
                artifact_report_reply["x"],
                artifact_report_reply["y"],
                artifact_report_reply["z"],
                artifact_report_reply["report_status"],
                artifact_report_reply["score_change"],
                http_status,
                http_reason,
            ]
            self.publishSubmissionReply(proposal_return)

            # remove the artifact from the book keeping
            # it may have already been removed if the artifact was deleted.
            if artifact_unique_id in self.queued_artifacts.keys():
                self.queued_artifacts.pop(artifact_unique_id)
                # clear the displayed image
                self.clear_displayed_img_pub.publish(Bool(True))

            self.submitted_artifacts[artifact_unique_id] = self.all_artifacts[
                artifact_unique_id
            ]

            # remove the artifact update message from the image visualizer plugin
            # if it is still visible
            update_msg = ArtifactVisualizerUpdate()
            update_msg.data = ArtifactVisualizerUpdate.HIDE
            self.update_label_pub.publish(update_msg)

            # nothing is being displayed, so reset bookeeping
            self.displayed_pose = [None, None, None]
            self.displayed_category = None
            self.displayed_artifact_id = None

            return True

        # if we got back nothing from DARPA, unlikely to happen...
        return False

    def publishSubmissionReply(self, proposal_return):
        """
        Publish the information returned by DARPA about our artifact submission

        proposal_return is a list containing info from darpa about oru artifact submission
        """
        [
            submission_time_raw,
            artifact_type,
            x,
            y,
            z,
            report_status,
            score_change,
            http_response,
            http_reason,
        ] = proposal_return

        # publish the message to display in thesubmission reply plugin
        reply_msg = ArtifactSubmissionReply()
        reply_msg.submission_time_raw = float(submission_time_raw)
        reply_msg.artifact_type = str(artifact_type)
        reply_msg.x = x
        reply_msg.y = y
        reply_msg.z = z
        reply_msg.report_status = str(report_status)
        reply_msg.score_change = score_change
        reply_msg.http_response = str(http_response)
        reply_msg.http_reason = str(http_reason)

        self.submission_reply_pub.publish(reply_msg)

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
        self.publishManipulationInfomation(msg.data)

    def publishManipulationInfomation(self, unique_id):
        """
        Send out the artifact manipulation data (original pose and current pose)

        unique_id is a string that is the unique_id for the artifact
        """
        manip_msg = self.guiArtifactToRos(self.all_artifacts[unique_id])
        self.manipulation_info_pub.publish(manip_msg)

    def getArtifactImage(self, msg):
        """
        Get another artifact image to show.

        msg.data defines whethere we go forward in the set or backward
        """
        direction = msg.data
        # check for errors with request
        update_msg = GuiMessage()

        if self.displayed_artifact_id == None:
            update_msg.data = (
                "No artifact has been selected. Please select one from the queue"
            )
            update_msg.color = update_msg.COLOR_ORANGE
            self.message_pub.publish(update_msg)
            return

        # display a black image because this artifact has no images
        img_count = len(self.all_artifacts[self.displayed_artifact_id].imgs)

        if img_count == 0:
            self.publishImgToDisplay(-1)
        elif direction == RetreiveArtifactImage.DIRECTION_FORWARD:
            if self.img_ind_displayed < img_count - 1:
                # we have some runway to go forward in the sequence
                self.img_ind_displayed += 1
            else:
                # we can't go forward anymore, loop back around
                self.img_ind_displayed = 0
            self.publishImgToDisplay(self.img_ind_displayed)

        elif direction == RetreiveArtifactImage.DIRECTION_BACKWARD:
            if self.img_ind_displayed > 0:
                # we have some runway to go backward in the sequence
                self.img_ind_displayed -= 1
            else:
                # we're already at the beginning on the sequence, loop back around
                self.img_ind_displayed = img_count - 1
            self.publishImgToDisplay(self.img_ind_displayed)

        # display the first image
        elif direction == RetreiveArtifactImage.DISPLAY_FIRST and img_count > 0:
            self.publishImgToDisplay(0)

        else:
            update_msg.data = "Somehow a wrong direction was sent. Image not changed."
            update_msg.color = update_msg.COLOR_RED
            self.message_pub.publish(update_msg)

    def publishImgToDisplay(self, ind):
        """
        Publish an image to be displayed in the artifact visualization panel

        ind is the indice in the artifact's image list that the image to be displayed is (this comment could use better grammar)
        """
        if self.displayed_artifact_id == None:
            update_msg.data = "No artifact selected.  Select one from the queue"
            update_msg.color = update_msg.COLOR_ORANGE
            self.message_pub.publish(update_msg)
            return

        msg = ArtifactDisplayImage()
        if ind == -1:  # this artifact contains no images display the blank black image
            rospack = rospkg.RosPack()
            path = rospack.get_path("basestation_gui_python")
            name = "{0}/src/black_img.png".format(path)
            img = cv2.imread(name)
            msg.img = self.br.cv2_to_imgmsg(img)
            msg.image_ind = 0
            msg.num_images = 0
        else:
            # publish the image
            img = self.all_artifacts[self.displayed_artifact_id].imgs[ind]
            msg.img = self.br.cv2_to_imgmsg(img)
            msg.image_ind = ind
            msg.num_images = len(self.all_artifacts[self.displayed_artifact_id].imgs)

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

    def archiveArtifact(self, msg):
        """
        Used when an artifact should be removed from the queue.
        May require a separate plugin to manage such items.

        msg is a string of the unique_id
        """
        # go find the artifact
        artifact_to_archive = self.all_artifacts[msg.data]
        if artifact_to_archive != None:
            self.archived_artifacts[artifact_to_archive.unique_id] = artifact_to_archive
            # default return value is None if key not found
            self.queued_artifacts.pop(artifact_to_archive.unique_id)

            # if the artifact has images displayed, we want to send a message syaing its been deleted
            if artifact_to_archive.unique_id == self.displayed_artifact_id:

                update_msg = ArtifactVisualizerUpdate()
                update_msg.data = ArtifactVisualizerUpdate.DELETE
                self.update_label_pub.publish(update_msg)

        else:
            update_msg = GuiMessage()
            update_msg.data = "Artifact not removed from handler"
            update_msg.color = update_msg.COLOR_RED
            self.message_pub.publish(update_msg)

    def bookeepAndPublishNewArtifact(self, artifact):
        """
        Add the new artifact to the various lists and publish the
        info that we have a new artofact to the various channels

        artifact is a GuiArtifact object
        """
        self.all_artifacts[artifact.unique_id] = artifact
        self.queued_artifacts[artifact.unique_id] = artifact

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
    ):

        self.category = category
        self.pose = pose
        self.orig_pose = copy.deepcopy(pose)
        self.source_robot_id = source_robot_id
        self.artifact_report_id = artifact_report_id
        self.time_from_robot = (
            time_from_robot if time_from_robot is not None else -1
        )  # time the detection has come in from the robot. TODO: change to be something different?
        self.time_to_darpa = -1  # time submitted to darpa
        self.unread = True
        self.darpa_response = ""
        self.imgs = imgs if imgs is not None else []
        self.img_stamps = (
            img_stamps if img_stamps is not None else []
        )  # time stamps of the images
        self.original_timestamp = original_timestamp
        self.unique_id = GuiArtifact.message_id(
            source_robot_id, artifact_report_id, original_timestamp
        )

    @staticmethod
    def message_id(robot_id, report_id, secs):
        return "{0}/{1}/{2}".format(robot_id, report_id, secs)

    @staticmethod
    def message_id_from_msg(msg):
        return GuiArtifact.message_id(
            msg.artifact_robot_id, msg.artifact_report_id, msg.artifact_stamp.secs
        )


if __name__ == "__main__":
    time.sleep(0.5)  # give the gui time to launch and setup the proper subscribers

    node = ArtifactHandler()
    node.run()
