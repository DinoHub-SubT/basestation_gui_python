#!/usr/bin/env python

"""
A node for getting status information (time, score, remaining reports) from DARPA
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University (CMU) / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebastion or Matt).
"""
import rospy
import rospkg
import tf
import threading
import time
import robots
import requests
import os
import json
import base64
import math

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped
from basestation_gui_python.msg import DarpaStatus, GuiMessage
from base_py import BaseNode

MAX_ERRORS = 3


class Transform(object):
    def __init__(self):
        self.position = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.orientation = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}

    def setTranslation(self, x, y, z):
        self.position = {"x": x, "y": y, "z": z}

    def setRotation(self, x, y, z, w):
        mag = math.sqrt(x*x + y*y + z*z + w*w)
        x = x / mag
        y = y / mag
        z = z / mag
        w = w / mag
        self.orientation = {"x": x, "y": y, "z": z, "w": w}

    def __str__(self):
        m = "Tx: ({0}, {1}, {2}) -- [{3}, {4}, {5}, {6}]"
        return m.format(
            self.position["x"],
            self.position["y"],
            self.position["z"],
            self.orientation["x"],
            self.orientation["y"],
            self.orientation["z"],
            self.orientation["w"],
        )


class DarpaBridge(BaseNode):
    def __init__(self):
        super(DarpaBridge, self).__init__("DarpaBridge")
        self.subscriptions = []

    def initialize(self):
        cfg = robots.Config()
        d = cfg.darpa
        http = "{0}:{1}{2}"
        auth = "Bearer {0}"

        self.status_uri = http.format(d.score_address, d.score_port, d.score_status_uri)
        self.cloud_uri = http.format(d.map_address, d.map_port, d.map_cloud_uri)
        self.pose_uri = http.format(d.map_address, d.map_port, d.map_pose_uri)
        self.status_headers = {"Authorization": auth.format(d.score_bearer_token)}
        self.map_headers = {"Authorization": auth.format(d.map_bearer_token)}
        self.status_thread = None
        self.error_count = 0
        self.last_modified = dict()
        self.transforms = dict()

        self.message_pub = rospy.Publisher(
            "/gui/message_print", GuiMessage, queue_size=10
        )
        self.status_pub = rospy.Publisher(
            "/gui/darpa_status", DarpaStatus, queue_size=10
        )

        def sub(topic, what, callback, robot):
            s = rospy.Subscriber(topic, what, callback, robot)
            self.subscriptions.append(s)

        for r in cfg.robots:
            self.last_modified[r.uuid] = None
            self.transforms[r.uuid] = Transform()
            self.getTransform(r)
            cloud = "/{0}/{1}".format(r.topic_prefix, r.topics.get("point_cloud"))
            pose = "/{0}/{1}".format(r.topic_prefix, r.topics.get("odometry"))
            sub(cloud, PointCloud2, self.onCloud, r)
            sub(pose, Odometry, self.onOdometry, r)

        # Start a schedule which runs "get status" every few seconds.
        self.connected = rospy.get_param("/connect_to_command_post")
        if self.connected:
            self.startThread(1.0)
        return True

    def startThread(self, howLong):
        self.status_thread = threading.Timer(howLong, self.getStatus)
        self.status_thread.start()

    def getStatus(self):
        """Queries the current score from the DARPA server."""
        try:
            MILLIS = 0.001
            req = requests.get(
                self.status_uri, headers=self.status_headers, timeout=300 * MILLIS
            )
            self.error_count = 0
            if req.ok:
                try:
                    resp = req.json()
                    s = DarpaStatus()
                    s.score = resp["score"]
                    s.time_elapsed = float(resp["run_clock"])
                    s.remaining_reports = resp["remaining_reports"]
                    self.status_pub.publish(s)
                except ValueError as e:
                    rospy.logerr("[DARPA Status] JSON encoding error: %s", e)
            else:
                m = "[DARPA Status] Bad request -- Code %s, Reason: %s"
                rospy.logerr(m, req.status_code, req.reason)
        except Exception as e:
            m = "[DARPA Status] GET failed [%d of %d]: %s"
            self.error_count += 1
            rospy.logerr(m, self.error_count, MAX_ERRORS, e)

        if self.error_count < MAX_ERRORS:
            self.startThread(1.0)
        else:
            m = "[DARPA Status] Too many errors encountered when querying scoring status from DARPA command post.  Will try again in 30 seconds."
            g = GuiMessage()
            g.data = m
            g.color = g.COLOR_ORANGE
            rospy.loginfo(m)
            self.message_pub.publish(g)
            self.startThread(30.0)

    def onCloud(self, pointCloud2, robot):
        if not self.connected:
            return
        fields = []
        for f in pointCloud2.fields:
            fields.append(
                {
                    "name": str(f.name),
                    "offset": int(f.offset),
                    "datatype": int(f.datatype),
                    "count": int(f.count),
                }
            )
        tx = self.getTransform(robot)
        cloud = {
            "type": "PointCloud2",
            "msg": {
                "header": {"stamp": rospy.get_time(), "frame_id": "darpa"},
                "origin": {"position": tx.position, "orientation": tx.orientation},
                "fields": fields,
                "is_bigendian": pointCloud2.is_bigendian,
                "point_step": pointCloud2.point_step,
                "data": base64.b64encode(pointCloud2.data),
            },
        }
        try:
            MILLIS = 0.001
            req = requests.post(
                self.cloud_uri,
                headers=self.map_headers,
                json=cloud,
                timeout=300 * MILLIS,
            )
            self.error_count = 0
            if not req.ok:
                m = "[DARPA Status] Bad request -- Code %s, Reason: %s"
                rospy.logerr(m, req.status_code, req.reason)
        except Exception as e:
            self.error_count += 1
            m = "[DARPA Status] POST point cloud failed: %s"
            rospy.logerr(m, e)

    def onOdometry(self, odometry, robot):
        if not self.connected:
            return

        tx = self.getTransform(robot)
        ts = TransformStamped()
        ts.header.frame_id = "darpa"
        ts.child_frame_id = robot.topic_prefix + "_map"
        ts.transform.translation.x = tx.position["x"]
        ts.transform.translation.y = tx.position["y"]
        ts.transform.translation.z = tx.position["z"]
        ts.transform.rotation.x = tx.orientation["x"]
        ts.transform.rotation.y = tx.orientation["y"]
        ts.transform.rotation.z = tx.orientation["z"]
        ts.transform.rotation.w = tx.orientation["w"]

        ps = PoseStamped()
        ps.pose = odometry.pose.pose
        ps.header.frame_id = ts.child_frame_id

        tfx = tf.TransformerROS()
        tfx.setTransform(ts)

        ps = tfx.transformPose(ts.header.frame_id, ps)
        msg = {
            "header": {"stamp": rospy.get_time(), "frame_id": "darpa"},
            "poses": [
                {
                    "name": robot.name,
                    "position": {
                        "x": ps.pose.position.x,
                        "y": ps.pose.position.y,
                        "z": ps.pose.position.z,
                    },
                    "orientation": {
                        "x": ps.pose.orientation.x,
                        "y": ps.pose.orientation.y,
                        "z": ps.pose.orientation.z,
                        "w": ps.pose.orientation.w,
                    },
                }
            ],
        }
        try:
            MILLIS = 0.001
            req = requests.post(
                self.pose_uri, headers=self.map_headers, json=msg, timeout=300 * MILLIS
            )
            self.error_count = 0
            if not req.ok:
                m = "[DARPA Status] Bad request -- Code %s, Reason: %s"
                rospy.logerr(m, req.status_code, req.reason)
        except Exception as e:
            self.error_count += 1
            m = "[DARPA Status] POST robot pose failed: %s"
            rospy.logerr(m, e)

    def getTransform(self, robot):
        """
        Returns the current transform for the robot, which is updated if a recent
        calibration has been taken.
        """
        path = rospkg.RosPack().get_path("basestation_gui_python")
        path = os.path.join(path, "data", "calibration", robot.uuid + ".json")
        stat = None
        tx = Transform()

        try:
            stat = os.lstat(path)
        except Exception:
            tx.setTranslation(0, 0, 0)
            tx.setRotation(0, 0, 0, 1)
            self.last_modified[robot.uuid] = None
            self.transforms[robot.uuid] = tx
            return tx

        last = self.last_modified.get(robot.uuid)
        if last == None or stat.st_mtime > last:
            try:
                with open(path, "r") as f:
                    d = json.load(f)
                    q = d["quaternion"]
                    t = d["transform"]
                    tx.setTranslation(t[0][3], t[1][3], t[2][3])
                    tx.setRotation(q[0], q[1], q[2], q[3])
                    self.last_modified[robot.uuid] = stat.st_mtime
            except Exception:
                tx.setTranslation(0, 0, 0)
                tx.setRotation(0, 0, 0, 1)
                self.last_modified[robot.uuid] = None
            self.transforms[robot.uuid] = tx

        return self.transforms[robot.uuid]

    def execute(self):
        return True

    def shutdown(self):
        if self.status_thread:
            self.status_thread.cancel()
        for s in self.subscriptions:
            s.unregister()
        rospy.loginfo("[DARPA Status] shutting down")


if __name__ == "__main__":
    node = DarpaBridge()
    node.run()
