#!/usr/bin/python
import rospy
from std_msgs.msg import String
import yaml
import sys
from geometry_msgs.msg import PoseStamped
import pdb
from basestation_gui_python.msg import RadioMsg
import numpy as np

class RosGuiBridge:
    def __init__(self, config_filename):

        # parse the config file
        config = yaml.load(open(config_filename, 'r').read())
        
        #read info on the experiment parameters (# of robots, etc.)
        exp_params = config['experiment_params']
        self.robot_names = [] #robot names corresponding to topic published
        self.robot_commands = [] #different types of estop commands

        for name in exp_params['robot_names']:
            self.robot_names.append(name)

        #robot names should be unique
        if(len(np.unique(self.robot_names))!=len(self.robot_names)):
        	raise ValueError('Not all of the robot names are unique!')

        for command in exp_params['robot_commands']:
            self.robot_commands.append(command)

        self.estop_commands = self.robot_commands[:4]

        #define radio message publisher
        self.radio_pub = rospy.Publisher('/from_gui', RadioMsg, queue_size=50) #queue_size arbitraily chosen

        #subscriber for listening to waypoint goals
        self.waypoint_topic = "move_base_simple/goal" #topic for listening to BSM-defined waypoints

        self.waypoint_listeners = []
        for i in range(len(self.robot_names)):
        	self.waypoint_listeners.append(rospy.Subscriber(self.waypoint_topic, PoseStamped, self.publishWaypointGoal, ""))
        	self.waypoint_listeners[-1].unregister() #don't start subscribing quite yet

        
    def publishRobotCommand(self, command, robot_name):
        '''
        A command button has been pressed. Publish a command from the gui to the robot
        '''

        #de-register the waypoint listener because possibly another type of button has been pressed
    	self.waypoint_listeners[self.robot_names.index(robot_name)].unregister()
        
        if(command in self.estop_commands):
            self.publishEstop(command, robot_name)
        elif(command == "Define waypoint"):
            self.defineWaypoint(robot_name)
        elif(command == "Return home"):
            self.publishReturnHome(robot_name)
        else:
            print 'WARNING: Button pressed does not have a function call associated with it!'

    def publishEstop(self, command, robot_name):
        '''
        Publish an estop message after a button has been pressed
        '''

        radio_msg = RadioMsg()
        msg.message_type = 1
        radio_msg.recipient_robot_id = self.robot_names.index(robot_name)
        if(command=="Resume"):
            radio_msg.data = "0"
        elif(command=="Pause"):
            radio_msg.data = "1"
        elif(command=="Soft e-stop"):
            radio_msg.data = "2"
        elif(command=="Hard e-stop"):
            radio_msg.data = "3"
        else:
            print 'WARNING: The pressed button does not correspond to an estop command the Bridge knows about'

        self.radio_pub.publish(radio_msg)

    def publishReturnHome(self, robot_name):
        '''
        Send out a message for the robot to return home
        '''

        pass

    def publishWaypointGoal(self, msg, robot_name):

    	radio_msg = RadioMsg()
        radio_msg.message_type = 2
        radio_msg.recipient_robot_id = self.robot_names.index(robot_name)
        radio_msg.data = str(msg.pose.position.x) +","+str(msg.pose.position.y) +","+str(msg.pose.position.z)+","+\
        				 str(msg.pose.orientation.x) +","+str(msg.pose.orientation.y) +","+str(msg.pose.orientation.z)+","+\
        				 str(msg.pose.orientation.w)

        self.radio_pub.publish(radio_msg)

        print radio_msg.data



    def defineWaypoint(self, robot_name):
        '''
        Listen for a waypoint to be pressed in Rviz
        '''

        #subscriber for listening to waypoint goals
        self.waypoint_listeners[self.robot_names.index(robot_name)] = rospy.Subscriber(self.waypoint_topic, PoseStamped, self.publishWaypointGoal, robot_name)

        

    def changeControlButtonColors(self, control_buttons, robot_num, button_pressed):
        '''
        Change button colors to indicate which was pressed
        '''

        #set the color of the other estop buttons to gray
        for button in control_buttons[robot_num]:
            button.setStyleSheet("background-color: None")

        #set the color of the button pressed 
        button_pressed.setStyleSheet("background-color: red")




        

if __name__ == '__main__':
    rospy.init_node('ros_gui_bridge', anonymous=True)
    
    config_filename = rospy.get_param('~config', '')

    ros_gui_bridge = RosGuiBridge(config_filename)    

    rospy.spin()
