#!/usr/bin/env python

"""
Adopted heavily from tutorial code (basic_controls.py):
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

For processing the artifact refinement interactive marker. 
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebation or Matt).
"""

import rospy
import math

from base_py import BaseNode
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from tf.broadcaster import TransformBroadcaster
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, InteractiveMarkerControl


class InteractiveMarkerProcessing(BaseNode):
    """
    ROS node that is a bridge to the interactive marker placement in RViz.

    When the user places a navigation marker for the robot or wishes to refine a
    position of a detected artifact image within the GUI, a message will be received by
    this node to have the marker placed/removed within RViz.
    """

    def __init__(self):
        super(InteractiveMarkerProcessing, self).__init__("InteractiveMarkerProcessing")

    def initialize(self):
        self.subscriptions = []

        def sub(topic, what, callback):
            s = rospy.Subscriber(topic, what, callback)
            self.subscriptions.append(s)

        def wayptMarker(scale):
            m = Marker()
            m.type = Marker.CYLINDER
            m.scale.x = scale * 0.4
            m.scale.y = scale * 0.4
            m.scale.z = scale * 3
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 1.0
            return m

        def refineMarker(scale):
            m = Marker()
            m.type = Marker.SPHERE
            m.scale.x = scale * 0.4
            m.scale.y = scale * 0.4
            m.scale.z = scale * 0.4
            m.color.r = 1.0
            m.color.g = 165.0 / 255.0
            m.color.b = 0.0
            m.color.a = 1.0
            return m

        self.refinement = MarkerServer(
            "artifact_refinement", "Artifact Marker Refinement", refineMarker
        )
        self.waypoint = MarkerServer(
            "define_waypoint", "Robot Move Waypoint", wayptMarker
        )
        self.refinement.hide(None)
        self.waypoint.hide(None)

        sub("/refinement_marker_pos", Point, self.refinement.move)
        sub("/refinement_marker_off", Point, self.refinement.hide)
        sub("/define_waypoint_marker_pos", Point, self.waypoint.move)
        sub("/define_waypoint_marker_off", Point, self.waypoint.hide)
        return True

    def execute(self):
        return True

    def shutdown(self):
        self.refinement.shutdown()
        self.waypoint.shutdown()
        for s in self.subscriptions:
            s.unregister()
        rospy.loginfo("[Interactive Marker Processing] shutting down")


class MarkerServer(object):
    """
    MarkerServer acts as an intermediary between the interactive processing node and an
    InteractiveMarkerServer; thus, allowing the placement of interactive markers within
    RViz.
    """

    def __init__(self, serverName, description, createMarkerDisplay):
        """
        Creates a portal to an interactive marker server.

        serverName  - Name assigned to the server.
        description - Text description of the server that is displayed in RViz.

        createMarkerDisplay - A function that takes a floating point scale parameter
                              and is expected to return a Marker object that shows
                              how to display the interactive marker inside of RViz.
        """
        self.server = InteractiveMarkerServer(serverName)
        self.menu = MenuHandler()
        self.tfCaster = TransformBroadcaster()
        self.counter = 0
        self.ref_frame = rospy.get_param("~reference_frame")

        self.menu.insert("First Entry", callback=self.onFeedback)
        self.menu.insert("Second Entry", callback=self.onFeedback)

        sub = self.menu.insert("Submenu")
        self.menu.insert("First Entry", parent=sub, callback=self.onFeedback)
        self.menu.insert("Second Entry", parent=sub, callback=self.onFeedback)

        im = InteractiveMarker()
        im.header.frame_id = self.ref_frame
        im.pose.position = Point(0, 0, 0)
        im.scale = 1
        im.name = "simple_6dof_MOVE_3D"
        im.description = description

        md = createMarkerDisplay(im.scale)
        ic = InteractiveMarkerControl()
        ic.always_visible = True
        ic.markers.append(md)
        im.controls.append(ic)
        im.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_3D

        self.timer = rospy.Timer(rospy.Duration(0.01), self.onFrame)
        self.marker = im
        self.server.applyChanges()

    def shutdown(self):
        self.timer.shutdown()

    def onFeedback(self, feedback):
        self.server.applyChanges()

    def onFrame(self, msg):
        now = rospy.Time.now()
        v3 = (0, 0, math.sin(self.counter / 140.0) * 2.0)
        v4 = (0, 0, 0, 1.0)
        self.counter += 1
        self.tfCaster.sendTransform(v3, v4, now, self.ref_frame, "moving_frame")

    def move(self, msg):
        """
        Informs the marker server move the interactive marker to a specific position.
        """
        self.marker.pose.position = msg
        self.server.insert(self.marker, self.onFeedback)
        self.menu.apply(self.server, self.marker.name)
        self.server.applyChanges()

    def hide(self, msgIgnored):
        """Forces the marker server to hide the interactive marker."""
        self.server.erase(self.marker.name)
        self.server.applyChanges()


if __name__ == "__main__":
    node = InteractiveMarkerProcessing()
    node.run()
