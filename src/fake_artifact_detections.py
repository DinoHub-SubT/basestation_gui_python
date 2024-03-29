#!/usr/bin/env python

"""
File which publishes fake artifact detections
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebation or Matt).
"""
from __future__ import print_function

import rospy
import pdb
import random
import time

from basestation_msgs.msg import WifiDetection, StatusUpdate
from std_msgs.msg import String
from nav_msgs.msg import Odometry

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import rospkg

# pdb.set_trace()


class FakePublisher:
    def __init__(self):

        rospy.init_node("fake_artifact_node", anonymous=True)

        self.artifact_pub = rospy.Publisher(
            "/ugv1/wifi_detection", WifiDetection, queue_size=10
        )
        self.message_pub = rospy.Publisher(
            "/gui_message_listener", String, queue_size=10
        )

        self.artifact_types = [4, 3, 5, 1, 2]
        self.robot_nums = [0, 1]

        self.total_num_to_pub = 50  # 1000
        self.num_pubbed = 0

        # add stuff for testing ji button pipeline
        self.ji_pub_ground = rospy.Publisher(
            "/ugv1/integrated_to_map", Odometry, queue_size=10
        )
        self.total_pub = rospy.Publisher("/position", Odometry, queue_size=10)

        # add stuff to test status panel updates
        self.status_pub = rospy.Publisher(
            "/ugv1/status_update", StatusUpdate, queue_size=10
        )

        self.published_list = []

        self.deleted_ids = []

    def pub_msgs(self):

        # publish artifact_reports first

        if self.num_pubbed < self.total_num_to_pub * 0.2:
            # print "new artifact", time.time()
            self.pubArtifactReport(update=False)

        else:  # update some artifacts
            # print "updated artifact", time.time()
            self.pubArtifactReport(update=True)

        # print self.deleted_ids

        # publish all of the other message types
        self.ji_pub_ground.publish(self.getJiFakePose())
        self.total_pub.publish(self.getTotalFakePose())
        self.status_pub.publish(self.getStatusMsg())

        if random.random() < 0.1:
            self.message_pub.publish("Message system testing")

    def pubArtifactReport(self, update):
        if not update:  # generate a new detection
            typ = random.sample(self.artifact_types, 1)[0]
            report_id = self.num_pubbed  # random.randint(0,9999)
            robot_id = random.sample(self.robot_nums, 1)[0]
            timestamp = time.time()
            msg = self.getFakeWifiMsg(
                artifact_report_id=report_id,
                artifact_type=typ,
                artifact_robot_id=robot_id,
                artifact_pos=[
                    random.random() * 5.0,
                    random.random() * 5.0,
                    random.random() * 5.0,
                ],
                timestamp=timestamp,
            )
        else:  # update an existing detection
            rand_ind = random.randint(0, len(self.published_list) - 1)
            robot_id = self.published_list[rand_ind][0]
            report_id = self.published_list[rand_ind][1]
            timestamp = self.published_list[rand_ind][2]
            if (report_id not in self.deleted_ids) and (random.random() < 0.0):
                typ = WifiDetection.ARTIFACT_REMOVE
                print("wifi  delete:", typ, robot_id, report_id)
            else:
                typ = random.sample(self.artifact_types, 1)[0]

            msg = self.getFakeWifiMsg(
                artifact_report_id=report_id,
                artifact_type=typ,
                artifact_robot_id=robot_id,
                artifact_pos=[random.random() * 5.0, 0, 1.2],
                timestamp=timestamp,
            )

            if msg.artifact_report_id not in self.deleted_ids:
                if not update:
                    print(
                        "new wifim: ",
                        msg.artifact_type,
                        msg.artifact_robot_id,
                        msg.artifact_report_id,
                    )
                else:
                    print(
                        "wifim update: ",
                        msg.artifact_type,
                        msg.artifact_robot_id,
                        msg.artifact_report_id,
                    )

                self.published_list.append(
                    [
                        msg.artifact_robot_id,
                        msg.artifact_report_id,
                        msg.artifact_stamp.secs,
                    ]
                )
                self.img_pub.publish(msg)

            if msg.artifact_type == WifiDetection.ARTIFACT_REMOVE:
                self.deleted_ids.append(msg.artifact_report_id)

    def getJiFakePose(self):
        ji_msg = Odometry()

        ji_msg.pose.pose.position.x = 1 + random.random()
        ji_msg.pose.pose.position.y = 2 + random.random()
        ji_msg.pose.pose.position.z = 3.14 + random.random()

        return ji_msg

    def getTotalFakePose(self):
        ji_msg = Odometry()

        ji_msg.pose.pose.position.x = 1 + random.random()
        ji_msg.pose.pose.position.y = 2 + random.random()
        ji_msg.pose.pose.position.z = 3.14 + random.random()

        return ji_msg

    def getStatusMsg(self):
        statuses = [
            StatusUpdate.WHAT_BATTERY,
            StatusUpdate.WHAT_COMMS,
            StatusUpdate.WHAT_MOBILITY,
            StatusUpdate.WHAT_CPU,
            StatusUpdate.WHAT_DISK_SPACE,
        ]
        severities = [
            StatusUpdate.SEVERITY_OK,
            StatusUpdate.SEVERITY_WARNING,
            StatusUpdate.SEVERITY_CRITICAL,
        ]
        values = [1, 4, 9, 10]

        status_msg = StatusUpdate()
        status_msg.what = random.sample(statuses, 1)[0]
        status_msg.value = str(random.sample(values, 1)[0])
        status_msg.severity = random.sample(severities, 1)[0]
        return status_msg

    def getFakeWifiMsg(
        self,
        artifact_report_id,
        artifact_type,
        artifact_robot_id,
        artifact_pos,
        timestamp,
    ):
        def img_path(name):
            gui = rospkg.RosPack().get_path("basestation_gui_python")
            return "{0}/fake_artifact_imgs/{1}".format(gui, name)

        # 'survivor', 'fire extinguisher', 'phone', 'backpack', 'drill'
        # [4, 3, 5, 1, 2]
        if artifact_type == 3:
            image_filename = img_path("test_img.jpg")
        elif artifact_type == 4:
            image_filename = img_path("human.jpg")
        elif artifact_type == 2:
            image_filename = img_path("drill.jpg")
        elif artifact_type == 1:
            image_filename = img_path("backpack.jpg")
        elif artifact_type == 5:
            image_filename = img_path("cell_phone.jpg")
        elif artifact_type == WifiDetection.ARTIFACT_REMOVE:
            image_filename = img_path("cell_phone.jpg")

        img = cv2.imread(image_filename)
        br = CvBridge()
        img = br.cv2_to_imgmsg(img)
        img.header.stamp.secs = timestamp

        if artifact_report_id == None:  # we need to generate some fake data
            msg = None
        else:
            msg = WifiDetection()
            msg.imgs = [img] * random.randint(0, 4)
            msg.artifact_robot_id = artifact_robot_id
            msg.artifact_report_id = artifact_report_id
            msg.artifact_type = artifact_type
            msg.artifact_x = artifact_pos[0]
            msg.artifact_y = artifact_pos[1]
            msg.artifact_z = artifact_pos[2]
            msg.artifact_stamp.secs = timestamp
        return msg


if __name__ == "__main__":

    try:
        fake_publisher = FakePublisher()
        fake_publisher.rate = rospy.Rate(2.0)  # (5./3600.) #rate in hz

        while (
            not rospy.is_shutdown()
            and fake_publisher.num_pubbed < fake_publisher.total_num_to_pub
        ):
            fake_publisher.pub_msgs()

            fake_publisher.rate = rospy.Rate(random.random() * 5 + 1.0)

            fake_publisher.rate.sleep()

            fake_publisher.num_pubbed += 1

    except rospy.ROSInterruptException:
        pass
