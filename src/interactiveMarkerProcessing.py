#!/usr/bin/python

"""
Adopted heavily from tutorial code (basic_controls.py):
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

For processing the artifact refinement interactive marker. 
Contact: Bob DeBortoli (debortor@oregonstate.edu)
"""

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster

from random import random
from math import sin


class CustomInteractiveMarker:
    '''
    Class handles the interactive markers for artifact refinement
    '''
    def __init__(self, position):
        
        rospy.init_node("basic_controls")

        self.server = None
        self.menu_handler = MenuHandler()
        self.br =  TransformBroadcaster()
        self.counter = 0
        self.ref_frame = '/map'

        #setup a subscriber listenting to position changes  
        rospy.Subscriber('/refinement_marker_pos', Point, self.moveInteractiveMarkerAndShow)  

        #subscriber for hiding the refinement marker
        rospy.Subscriber('/refinement_marker_off', Point, self.hideMarkerCallback)      

        self.generateInteractiveMarker(position)
        self.hideInteractiveMarker()

    def generateInteractiveMarker(self, position):
        '''
        Custom function to initialzie a custom marker.
        Should only be called once at the beginning of gui intiialization
        '''
        
        # create a timer to update the published transforms
        rospy.Timer(rospy.Duration(0.01), self.frameCallback)

        self.server = InteractiveMarkerServer("basic_controls")

        self.menu_handler.insert( "First Entry", callback = self.processFeedback )
        self.menu_handler.insert( "Second Entry", callback = self.processFeedback )
        self.sub_menu_handle = self.menu_handler.insert( "Submenu" )
        self.menu_handler.insert( "First Entry", parent=self.sub_menu_handle, callback = self.processFeedback )
        self.menu_handler.insert( "Second Entry", parent=self.sub_menu_handle, callback = self.processFeedback )
      
        self.int_marker = self.make6DofMarker( False, InteractiveMarkerControl.MOVE_3D, position, False)

        self.server.applyChanges()

        rospy.spin()


    def frameCallback(self, msg):
        time = rospy.Time.now()
        self.br.sendTransform( (0, 0, sin(self.counter/140.0)*2.0), (0, 0, 0, 1.0), time, self.ref_frame, "moving_frame" )
        self.counter += 1

    def processFeedback(self, feedback):
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"

        mp = ""
        if feedback.mouse_point_valid:
            mp = " at " + str(feedback.mouse_point.x)
            mp += ", " + str(feedback.mouse_point.y)
            mp += ", " + str(feedback.mouse_point.z)
            mp += " in frame " + feedback.header.frame_id

        # if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        #     rospy.loginfo( s + ": button click" + mp + "." )
        # elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        #     rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
        # elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        #     rospy.loginfo( s + ": pose changed")
        # elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        #     rospy.loginfo( s + ": mouse down" + mp + "." )
        # elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        #     rospy.loginfo( s + ": mouse up" + mp + "." )

        self.server.applyChanges()


    def makeBox(self, msg):
        marker = Marker()

        marker.type = Marker.CUBE
        marker.scale.x = msg.scale * 0.45
        marker.scale.y = msg.scale * 0.45
        marker.scale.z = msg.scale * 0.45
        marker.color.r = 1.
        marker.color.g = 0.
        marker.color.b = 0.
        marker.color.a = 1.0


        return marker

    def makeBoxControl(self, msg):
        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append( self.makeBox(msg) )
        msg.controls.append( control )
        return control

    def make6DofMarker(self, fixed, interaction_mode, position, show_6dof = False):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.ref_frame  
        int_marker.pose.position = position
        int_marker.scale = 1

        int_marker.name = "simple_6dof"
        int_marker.description = "Simple 6-DOF Control"

        # insert a box
        marker_control = self.makeBoxControl(int_marker)
        int_marker.controls[0].interaction_mode = interaction_mode

        if fixed:
            int_marker.name += "_fixed"
            int_marker.description += "\n(fixed orientation)"

        if interaction_mode != InteractiveMarkerControl.NONE:
            control_modes_dict = { 
                              InteractiveMarkerControl.MOVE_3D : "MOVE_3D",
                              InteractiveMarkerControl.ROTATE_3D : "ROTATE_3D",
                              InteractiveMarkerControl.MOVE_ROTATE_3D : "MOVE_ROTATE_3D" }
            int_marker.name += "_" + control_modes_dict[interaction_mode]
            int_marker.description = "Artifact Refinement Marker"
            # if show_6dof: 
            #   int_marker.description += " + 6-DOF controls"
            # int_marker.description += "\n" + control_modes_dict[interaction_mode]

        # self.server.insert(int_marker, self.processFeedback)
        # self.menu_handler.apply( self.server, int_marker.name )

        return int_marker




    def moveInteractiveMarkerAndShow(self, msg):
        '''
        Move the interactive marker to a specific position
        '''

        self.int_marker.pose.position = msg

        self.server.insert(self.int_marker, self.processFeedback)
        self.menu_handler.apply( self.server, self.int_marker.name )
        self.server.applyChanges()

        

    def hideInteractiveMarker(self):
        '''
        Function to hide interactive marker
        '''

        self.server.erase(self.int_marker.name)
        self.server.applyChanges()

    def hideMarkerCallback(self, msg):
        self.hideInteractiveMarker()
    


if __name__=="__main__":

    pose = Point(0,0,0)
    CustomInteractiveMarker(pose)

#     generateInteractiveMarker()
#     # hideInteractiveMarker()
#     # rospy.spin()

#     rate = rospy.Rate(0.5)

#     while not rospy.is_shutdown():

#         if(server.get(int_marker.name)!=None):
#             hideInteractiveMarker()

#         else:
#             pose = Point( random()*3,random()*3,random()*3 )
#             moveInteractiveMarker(pose)
#         rate.sleep()

