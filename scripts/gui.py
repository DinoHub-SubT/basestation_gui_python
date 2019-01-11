#!/usr/bin/env python
import Tkinter as tk
from Tkinter import Tk, Label, Button, Frame
import pdb

from gui_to_ros import pub_estop
import rospy
from std_msgs.msg import Int32


# pdb.set_trace()

class CommandGUI:
    def __init__(self, master):
        self.master = master
        master.title("COMMAND CENTER")

        #setup the various publishers
        self.setupPubs()

        #setup the various sub-windows
        self.setupCommandFrame(master, pubs = [self.estop_pub])

    def setupPubs(self):
        self.estop_pub = rospy.Publisher('/e_stop', Int32, queue_size=10)
    
    def setupCommandFrame(self, master, pubs):
        '''
        The frame for sending commands to the robots
        '''

        #publishers to be used
        [estop_pub] = pubs

        self.command_frame = Frame(master)
        self.command_frame.grid()

        #e-stop for every robot
        #TODO: make flexible in number of robots
        self.estop_buts = []
        #TODO: add window pop-up to this method to check fi they really want to hard e-stop
        self.estop_buts.append(Button(self.command_frame, text="Hard e-stop", command = lambda: pub_estop(3, estop_pub)))
        self.estop_buts.append(Button(self.command_frame, text="Soft e-stop", command = lambda: pub_estop(2, estop_pub))) 
        self.estop_buts.append(Button(self.command_frame, text="Pause", command = lambda: pub_estop(1, estop_pub))) 
        self.estop_buts.append(Button(self.command_frame, text="Resume", command = lambda: pub_estop(0, estop_pub)))  

        for button in self.estop_buts:
            button.grid()


