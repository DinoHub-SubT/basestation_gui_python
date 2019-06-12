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

from basestation_gui_python.msg import DarpaStatus
from darpa_command_post.TeamClient import TeamClient

from base_py import BaseNode


class DarpaBridge(BaseNode):
    def __init__(self):
        super(DarpaBridge, self).__init__("DarpaBridge")

    def initialize(self):

        darpa = robots.Config().darpa

        self.auth_bearer_token = darpa.auth_bearer_token
        self.request_info_uri = darpa.request_info_uri

        # have an initial darpa status update
        self.darpa_status_update = {}
        self.darpa_status_update["run_clock"] = 999
        self.darpa_status_update["score"] = None
        self.darpa_status_update["remaining_reports"] = None

        # if we're also simulating the darpa command post
        self.connect_to_command_post = rospy.get_param("/connect_to_command_post")

        # start a schedule which runs "get status" every few seconds
        if self.connect_to_command_post:
            self.http_client = TeamClient()
            self.get_status_thread = threading.Timer(1.0, self.getStatus)
            self.get_status_thread.start()
            self.status_pub = rospy.Publisher(
                "/gui/darpa_status", DarpaStatus, queue_size=10
            )

        return True

    def getStatus(self):
        """
	Function that calls the http code to get the status and published to appropriate ros topic
        """
        self.darpa_status_update = self.http_client.get_status_from_command_post()

        msg = DarpaStatus()
        msg.time_elapsed = float(self.darpa_status_update["run_clock"])
        msg.score = self.darpa_status_update["score"]
        msg.remaining_reports = self.darpa_status_update["remaining_reports"]
        self.status_pub.publish(msg)

        # restart the thread to call this function again
        self.get_status_thread = threading.Timer(1.0, self.getStatus)
        self.get_status_thread.start()

    def shutdownHttpServer(self):
        """Closes the http connection."""
        # stop the thread that keeps reuesting the status
        self.get_status_thread.cancel()
        self.http_client.exit()

    def execute(self):
        return True

    def shutdown(self):
        self.shutdownHttpServer()
        rospy.loginfo("DarpaBridge node shutting down")


if __name__ == "__main__":
    node = DarpaBridge()
    node.run()
