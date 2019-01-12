#!/usr/bin/env python

from std_msgs.msg import Int32
import pdb

#pdb.set_trace()

class GuiToRos:
    '''
    Class to convert GUI code to ROS information
    '''
    def __init__(self, command_gui):
        self.command_gui = command_gui

    def pub_estop(self, number, pubs):
        '''
        Function to publish estop message (an int) after a button press
        '''
        #TODO: add window pop-up to this method to check if they really want to hard e-stop
        msg_int = Int32()
        msg_int.data = number
        pubs.publish(msg_int)

    def bot_status_callback(self, msg):
        '''
        Callback to display status information of the robots
        '''
        msg_type = msg._type
        robot_num, status_num = -1, -1

        #find which robot we should be referring to
        for i, bot_label in enumerate(self.command_gui.bot_labels):
            if(msg_type.find(bot_label)!=-1):
                robot_num = i



        #find which status we should be referring to
        for i, status_label in enumerate(self.command_gui.bot_status_types):
            if(msg_type.find(status_label)!=-1):
                status_num = i

        #do something based on what we know
        if(status_num==0): #we're dealing with a battery update
            self.command_gui.bot_status_labels[robot_num][status_num].config(bg="green")

        pdb.set_trace()

    