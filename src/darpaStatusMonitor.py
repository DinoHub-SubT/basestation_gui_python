#!/usr/bin/env python

"""
A node for getting status information (time, score, remaining reports) from DARPA
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University (CMU) / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebastion or Matt).
"""
import rospy
import threading
import time
import robots
import requests

from basestation_gui_python.msg import DarpaStatus, GuiMessage
from base_py import BaseNode

MAX_ERRORS = 3


class DarpaBridge(BaseNode):
    def __init__(self):
        super(DarpaBridge, self).__init__("DarpaBridge")

    def initialize(self):
        d = robots.Config().darpa
        self.uri = "{0}:{1}{2}".format(d.ip_address, d.port, d.scoring_uri)
        self.headers = {"Authorization": "Bearer {0}".format(d.auth_bearer_token)}
        self.status_thread = None
        self.error_count = 0

        # Start a schedule which runs "get status" every few seconds.
        connect = rospy.get_param("/connect_to_command_post")
        if not connect:
            return True

        self.message_pub = rospy.Publisher(
            "/gui/message_print", GuiMessage, queue_size=10
        )
        self.status_pub = rospy.Publisher(
            "/gui/darpa_status", DarpaStatus, queue_size=10
        )
        self.startThread()
        return True

    def startThread(self):
        self.status_thread = threading.Timer(1.0, self.getStatus)
        self.status_thread.start()

    def getStatus(self):
        """Queries the current score from the DARPA server."""
        try:
            req = requests.get(self.uri, headers=self.headers)
            if req.ok:
                self.error_count = 0
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
            self.startThread()
        else:
            m = "[DARPA Status] Too many errors encountered when querying scoring status from DARPA command post.  Restart Basestation Application to try again."
            g = GuiMessage()
            g.data = m
            g.color = g.COLOR_ORANGE
            self.message_pub.publish(g)
            rospy.loginfo(m)

    def execute(self):
        return True

    def shutdown(self):
        if self.status_thread:
            self.status_thread.cancel()
        rospy.loginfo("[Darpa Status Monitor] shutting down")


if __name__ == "__main__":
    node = DarpaBridge()
    node.run()
