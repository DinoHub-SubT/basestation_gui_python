#!/usr/bin/env python
import Tkinter as tk
    


# license removed for brevity
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String
from mavros_msgs.msg import Mavlink #sudo apt-get install ros-kinetic-mavros


global root
global pub
global pub2
global pub3
global button1
global button2
global button3
global button4
global state
global button_bg_colour

from Tkinter import Tk
from gui import CommandGUI



   

def close_gui():
    if rospy.is_shutdown():
        root.destroy()
    root.after(50, close_gui) # call again in 50 ms



if __name__ == '__main__':
    #TODO: insert license at beginning of every file

    rospy.init_node('command_gui', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    #initialize the command gui
    bot_labels = ['ground1', 'aerial1']
    root = Tk()
    command_gui = CommandGUI(root, bot_labels)
    root.after(50, close_gui) # 50ms timer. if rospy shuts down, close gui
    root.mainloop()

    


