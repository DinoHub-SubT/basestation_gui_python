#!/usr/bin/env python
import Tkinter as tk
from Tkinter import Tk, Label, Button, Frame
from ttk import Notebook
import pdb

import rospy
from std_msgs.msg import Int32
from functools import partial

from gui_to_ros import GuiToRos


# pdb.set_trace()

class CommandGUI:
    '''
    Place where the GUI code is help for sending commands to robots and displaying their information
    '''
    def __init__(self, master, bot_labels):
        self.master = master
        master.title("COMMAND CENTER")

        self.num_bots = len(bot_labels) #number of robots
        self.bot_labels = bot_labels #names to be displayed. also corresponds to topic name
        self.bot_status_types = ['Battery', 'Mobility', 'Comms'] #the standard types of status updates displayed for each robot

        #setup some utils file
        self.to_ros = GuiToRos(self)

        #setup the various pubs/subs
        self.setupPubs()

        #setup the various sub-windows
        self.setupCommandFrame(master)
        self.setupStatusFrame(master)

    def setupPubs(self):
        self.estop_pubs = []
        #make custom publishers for each robot
        for robot_label in self.bot_labels:
            self.estop_pubs.append(rospy.Publisher('/'+robot_label+'/e_stop', Int32, queue_size=10))

    
    
    def setupCommandFrame(self, master):
        '''
        The frame for sending commands to the robots
        '''
        self.command_frame = Frame(master)
        self.command_frame.grid(row=1, column=0)

        Label(self.command_frame, text = "Robot Controls").grid(row=0)

        self.command_notebook = Notebook(self.command_frame)
        self.command_notebook.grid(row=1, column=0)

        

        self.command_tabs = []
        for i in range(self.num_bots):
            self.command_tabs.append(Frame(self.command_notebook))

        for i, tab in enumerate(self.command_tabs):
            self.command_notebook.add(tab, text = self.bot_labels[i])

        #e-stop for every robot
        #TODO: make flexible in number of robots
        self.estop_buts = []
        for i in range(self.num_bots):
            self.estop_buts.append([])
        
        #TODO: add window pop-up to this method to check if they really want to hard e-stop
        for i in range(self.num_bots): #note the use of partial() to send arguments via dynamic variables (vars we iterate over)!!
            self.estop_buts[i].append(Button(self.command_tabs[i], text="Hard e-stop", command = partial(self.to_ros.pub_estop,3, self.estop_pubs[i])))
            self.estop_buts[i].append(Button(self.command_tabs[i], text="Soft e-stop", command = partial(self.to_ros.pub_estop,3, self.estop_pubs[i]))) 
            self.estop_buts[i].append(Button(self.command_tabs[i], text="Pause", command = partial(self.to_ros.pub_estop,3, self.estop_pubs[i]))) 
            self.estop_buts[i].append(Button(self.command_tabs[i], text="Resume", command = partial(self.to_ros.pub_estop,3, self.estop_pubs[i]))) 
 

        for button_set in self.estop_buts: #for the buttons of every robot
            for button in button_set: #for the button of a single robot
                button.grid()

    def setupStatusFrame(self, master):
        '''
        The frame for display information, both robot and competition related
        '''
        self.status_frame = Frame(master)
        self.status_frame.grid(row=0, column=0)

        Label(self.status_frame, text = "Robot Status").grid()


        #generate a status box for each robot
        self.bot_status_frames = []
        self.bot_status_labels = []
        for i in range(self.num_bots):
            self.bot_status_labels.append([])

        for i in range(self.num_bots):
            robot_frame = Frame(self.status_frame, highlightbackground='black', highlightthickness = 4)

            #add the appropriate labels
            for j in range(len(self.bot_status_types)):
                self.bot_status_labels[i].append(Label(robot_frame, text = self.bot_status_types[j]))

            for j in range(len(self.bot_status_labels[i])):
                self.bot_status_labels[i][j].grid()

            robot_frame.grid(row = 1, column = i)
            self.bot_status_frames.append(robot_frame)


        #setup the subscribers for those labels
        for robot_label in self.bot_labels:
            rospy.Subscriber('/'+robot_label+'/status_battery', Int32, self.to_ros.bot_status_callback)
            rospy.Subscriber('/'+robot_label+'/status_mobility', Int32, self.to_ros.bot_status_callback)
            rospy.Subscriber('/'+robot_label+'/status_comms', Int32, self.to_ros.bot_status_callback)





