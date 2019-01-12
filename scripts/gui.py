#!/usr/bin/env python
import Tkinter as tk
from Tkinter import Tk, Label, Button, Frame
from ttk import Notebook
import pdb

from gui_to_ros import pub_estop
import rospy
from std_msgs.msg import Int32
from functools import partial


# pdb.set_trace()

class CommandGUI:
    def __init__(self, master, bot_labels):
        self.master = master
        master.title("COMMAND CENTER")

        self.num_bots = len(bot_labels) #number of robots
        self.bot_labels = bot_labels #names to be displayed. also corresponds to topic name

        #setup the various publishers
        self.setupPubs()

        #setup the various sub-windows
        self.setupCommandFrame(master)

    def setupPubs(self):
        self.estop_pubs = []
        #make custom publishers for each robot
        for robot_label in self.bot_labels:
            print '/'+robot_label+'/e_stop'
            self.estop_pubs.append(rospy.Publisher('/'+robot_label+'/e_stop', Int32, queue_size=10))
    
    def setupCommandFrame(self, master):
        '''
        The frame for sending commands to the robots
        '''
        self.command_notebook = Notebook(master)
        self.command_notebook.grid()

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
            self.estop_buts[i].append(Button(self.command_tabs[i], text="Hard e-stop", command = partial(pub_estop,3, self.estop_pubs[i])))
            self.estop_buts[i].append(Button(self.command_tabs[i], text="Soft e-stop", command = partial(pub_estop,3, self.estop_pubs[i]))) 
            self.estop_buts[i].append(Button(self.command_tabs[i], text="Pause", command = partial(pub_estop,3, self.estop_pubs[i]))) 
            self.estop_buts[i].append(Button(self.command_tabs[i], text="Resume", command = partial(pub_estop,3, self.estop_pubs[i]))) 

        

        for button_set in self.estop_buts: #for the buttons of every robot
            for button in button_set: #for the button of a single robot
                button.grid()




