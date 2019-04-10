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

    def pubEstop(self, msg_data, robot_num, button_num):
        '''
        Function to publish estop message (an int) after a button press
        '''
        #TODO: add window pop-up to this method to check if they really want to hard e-stop

        #make the new button press be the only one that's colored
        default_color = '#d9d9d9'
        default_highlight_color = '#ececec'
        for button in self.command_gui.estop_buts[robot_num]:
            button.configure(bg = default_color)
            button.configure(activebackground = default_highlight_color)

        button_clicked = self.command_gui.estop_buts[robot_num][button_num]
        button_clicked.configure(bg = '#00FF00')
        button_clicked.configure(activebackground = button_clicked.cget('background'))

        #if its a hard estop, we want to double check with their decision
        if(button_clicked.cget('text')=='Hard e-stop'):
            self.command_gui.hard_estop_robot_num = robot_num
            self.command_gui.hard_estop_window.deiconify()
            self.command_gui.hard_estop_window_question_label.configure(text = "Are you sure you want to hard stop "+self.command_gui.bot_labels[robot_num]+"?" )
            self.command_gui.hard_estop_window_question_label.grid(row=0)
            

        else:
            msg_int = Int32()
            msg_int.data = msg_data
            self.command_gui.estop_pubs[robot_num].publish(msg_int)
            print "published!"


    def botStatusCallback(self, msg, topic_name):
        '''
        Callback to display status information of the robots
        '''

        #find which robot we should be referring to
        for i, bot_label in enumerate(self.command_gui.bot_labels):
            if(topic_name.find(bot_label)!=-1):
                robot_num = i


        #find which status we should be referring to
        for i, status_label in enumerate(self.command_gui.bot_status_types):
            if(topic_name.find(status_label)!=-1):
                status_num = i


        #do something based on what we know
        if(status_num==0): #we're dealing with a battery update
            self.command_gui.bot_status_labels[robot_num][status_num].config(bg="green")

    def hardEstop(self):
        '''
        Callback to send a hard estop command to a robot
        '''
        command_gui = self.command_gui
        msg_int = Int32()
        msg_int.data = 3
        command_gui.estop_pubs[command_gui.hard_estop_robot_num].publish(msg_int)
        command_gui.afterEstop()

    def hardEstopAllRobots(self):
        command_gui = self.command_gui
        msg_int = Int32()
        msg_int.data = 3
        for estop_pub in command_gui.estop_pubs: #loop over all of the robots
            estop_pub.publish(msg_int)


    