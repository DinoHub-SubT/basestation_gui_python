#!/usr/bin/python
import rospy
from std_msgs.msg import String
import yaml
import sys

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
        	raise ValueError('Not all of the robot names are unqiue!')

        for command in exp_params['robot_commands']:
            self.robot_commands.append(command)

        self.estop_commands = self.robot_commands[:4]

        #define radio message publisher
        self.radio_pub = rospy.Publisher('/from_gui', RadioMsg, queue_size=50) #queue_size arbitraily chosen

    def publishRobotCommand(self, command, robot_name):
        '''
        Publish a command from the gui to the robot
        '''

        if(command in self.estop_commands):
            self.publishEstop(command, robot_name)
        elif(command == "Define waypoint"):
            self.defineWaypoint(robot_num)
        elif(command == "Return home"):
            self.publishReturnHome(robot_num)
        else:
            print 'WARNING: Button pressed does not have a function call associated with it!'

    def publishEstop(self, command, robot_name):
        '''
        Publish an estop message after a button has been pressed
        '''

        msg = RadioMsg()
        msg.message_type = 1
        msg.recipient_robot_id = self.robot_names.index(robot_name)
        if(command=="Resume"):
            msg.data = "0"
        elif(command=="Pause"):
            msg.data = "1"
        elif(command=="Soft e-stop"):
            msg.data = "2"
        elif(command=="Hard e-stop"):
            msg.data = "3"
        else:
            print 'WARNING: The pressed button does not correspond to an estop command the Bridge knows about'

        self.radio_pub.publish(msg)

    def publishReturnHome(self, robot_num):
        '''
        Send out a message for the robot to return home
        '''

        pass

    def defineWaypoint(self, robot_num):
        '''
        Listen for a waypoint to be pressed in Rviz
        '''
        pass


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
