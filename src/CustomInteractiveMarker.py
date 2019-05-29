#!/usr/bin/env python

"""
Code for generating an interactive marker which can be used for artifact refinement. 
Adopted from tutorial code (willow garage) and Chao's C++ code. 

Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Contact: Bob DeBortoli (debortor@oregonstate.edu)
"""
from __future__ import print_function

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster
from math import sin


def frameCallback(msg, args):
    [counter, br] = args
    time = rospy.Time.now()
    br.sendTransform(
        (0, 0, sin(counter / 140.0) * 2.0),
        (0, 0, 0, 1.0),
        time,
        "base_link",
        "moving_frame",
    )
    counter += 1


def processFeedback(feedback, args):
    server = args

    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"

    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo(s + ": button click" + mp + ".")
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.loginfo(
            s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "."
        )
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo(s + ": pose changed")
    # TODO
    #          << "\nposition = "
    #          << feedback.pose.position.x
    #          << ", " << feedback.pose.position.y
    #          << ", " << feedback.pose.position.z
    #          << "\norientation = "
    #          << feedback.pose.orientation.w
    #          << ", " << feedback.pose.orientation.x
    #          << ", " << feedback.pose.orientation.y
    #          << ", " << feedback.pose.orientation.z
    #          << "\nframe: " << feedback.header.frame_id
    #          << " time: " << feedback.header.stamp.sec << "sec, "
    #          << feedback.header.stamp.nsec << " nsec" )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        rospy.loginfo(s + ": mouse down" + mp + ".")
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo(s + ": mouse up" + mp + ".")
    server.applyChanges()


def makeBox(msg):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker


def makeBoxControl(msg):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(makeBox(msg))
    msg.controls.append(control)
    return control


def make6DofMarker(
    fixed, interaction_mode, position, server, menu_handler, show_6dof=False
):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "map"
    int_marker.pose.position = position
    int_marker.scale = 0.5

    int_marker.name = "artifact_refinement_marker"
    int_marker.description = "Artifact Refinment Marker"

    # insert a box
    makeBoxControl(int_marker)
    int_marker.controls[0].interaction_mode = interaction_mode

    if fixed:
        int_marker.name += "_fixed"
        int_marker.description += "\n(fixed orientation)"

    if interaction_mode != InteractiveMarkerControl.NONE:
        control_modes_dict = {
            InteractiveMarkerControl.MOVE_3D: "MOVE_3D",
            InteractiveMarkerControl.ROTATE_3D: "ROTATE_3D",
            InteractiveMarkerControl.MOVE_ROTATE_3D: "MOVE_ROTATE_3D",
        }
        int_marker.name += "_" + control_modes_dict[interaction_mode]
        int_marker.description = "3D Control"
        if show_6dof:
            int_marker.description += " + 6-DOF controls"
        int_marker.description += "\n" + control_modes_dict[interaction_mode]

    server.insert(int_marker, processFeedback)
    menu_handler.apply(server, int_marker.name)


class CustomInteractiveMarker:
    """
    Class that is produces an interactive marker
    """

    def __init__(self):
        br = TransformBroadcaster()
        menu_handler = MenuHandler()
        counter = 0


if __name__ == "__main__":
    rospy.init_node("basic_controls")

    # create a timer to update the published transforms
    rospy.Timer(rospy.Duration(0.01), frameCallback, [counter, br])

    server = InteractiveMarkerServer("basic_controls")

    menu_handler.insert("First Entry", callback=processFeedback, args=(server))
    menu_handler.insert("Second Entry", callback=processFeedback, args=(server))
    sub_menu_handle = menu_handler.insert("Submenu")
    menu_handler.insert(
        "First Entry", parent=sub_menu_handle, callback=processFeedback, args=(server)
    )
    menu_handler.insert(
        "Second Entry", parent=sub_menu_handle, callback=processFeedback, args=(server)
    )

    position = Point(3, 0, 0)
    make6DofMarker(
        False, InteractiveMarkerControl.MOVE_3D, position, server, menu_handler, False
    )

    server.applyChanges()

    rospy.spin()
