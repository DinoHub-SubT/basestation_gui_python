#!/usr/bin/env python
import Tkinter as tk
<<<<<<< HEAD

import rospy
#from std_msgs.msg import Int32
#from std_msgs.msg import String
#from mavros_msgs.msg import Mavlink #sudo apt-get install ros-kinetic-mavros

from basestation_gui_python.msg import RadioMsg

global root
global pub_ros_to_teensy
global button1
global button2
global button3
global button4
global state
global button_bg_colour

def talker():

    global pub_ros_to_teensy
    pub_ros_to_teensy = rospy.Publisher('/ros_to_teensy', RadioMsg, queue_size=10)
    rospy.init_node('estop_gui', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    global root
    root = tk.Tk()
    frame = tk.Frame(root)
    frame.pack()

    back = tk.Frame(master=root, width=500, height=200)
    back.pack()

    global button1
    button1 = tk.Button(frame, 
                       text="hard e-stop", 
                       fg="black",
                       command=button_callback1)
    button1.pack(side=tk.LEFT)

    global button2
    button2 = tk.Button(frame, 
                       text="soft e-stop", 
                       fg="black",
                       command=button_callback2)
    button2.pack(side=tk.LEFT)

    global button3
    button3 = tk.Button(frame, 
                       text="pause", 
                       fg="black",
                       command=button_callback3)
    button3.pack(side=tk.LEFT)

    global button4
    button4 = tk.Button(frame, 
                       text="resume", 
                       fg="black",
                       command=button_callback4)
    button4.pack(side=tk.LEFT)


    button_quit = tk.Button(frame, 
                       text="QUIT", 
                       fg="red",
                       command=quit)
    button_quit.pack(side=tk.LEFT)

    global button_bg_colour
    button_bg_colour = button_quit.cget("background")

    root.after(50, check_exit) # 50ms timer before call function

    root.mainloop()	


    # while not rospy.is_shutdown():
    #     hello_str = "hello world %s" % rospy.get_time()
    #     rospy.loginfo(hello_str)
    #     pub.publish(hello_str)
    #     rate.sleep()

def check_exit():
    if rospy.is_shutdown():
        root.destroy()
    root.after(50, check_exit) # call again in 50 ms

def colour_state():
    button1.config(bg=button_bg_colour)
    button2.config(bg=button_bg_colour)
    button3.config(bg=button_bg_colour)
    button4.config(bg=button_bg_colour)
    if state==RadioMsg.ESTOP_HARD:
        button1.config(bg='red')
        button1.config(activebackground='red')
    if state==RadioMsg.ESTOP_SOFT:
        button2.config(bg='red')
        button2.config(activebackground='red')
    if state==RadioMsg.ESTOP_PAUSE:
        button3.config(bg='orange')
        button3.config(activebackground='orange')
    if state==RadioMsg.ESTOP_RESUME:
        button4.config(bg='green')
        button4.config(activebackground='green')   

def buttons_process(estop_data):
    print(estop_data)
    msg_radio_msg = RadioMsg()
    msg_radio_msg.message_type = RadioMsg.MESSAGE_TYPE_ESTOP
    msg_radio_msg.data = estop_data
    pub_ros_to_teensy.publish(msg_radio_msg)
    global state
    state = estop_data
    colour_state()

global window_are_you_sure

def button_callback1():
    # ask the user if they really want to hard e-stop
    global window_are_you_sure
    window_are_you_sure = tk.Toplevel(root)
    label = tk.Label(window_are_you_sure, text="Are you sure?")
    button_yes = tk.Button(window_are_you_sure, text="HARD STOP!!!", command=hard_stop_callback)
    button_yes.pack(side=tk.LEFT)
    button_no = tk.Button(window_are_you_sure, text="No, continue", command=close_window_are_you_sure)
    button_no.pack(side=tk.LEFT)

def hard_stop_callback():
    buttons_process(RadioMsg.ESTOP_HARD)
    window_are_you_sure.destroy()

def close_window_are_you_sure():
    window_are_you_sure.destroy()

def button_callback2():
    buttons_process(RadioMsg.ESTOP_SOFT)

def button_callback3():
    buttons_process(RadioMsg.ESTOP_PAUSE)

def button_callback4():
    buttons_process(RadioMsg.ESTOP_RESUME)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
=======
from Tkinter import Tk, Label, Button, Frame
from ttk import Notebook
import pdb

from gui_to_ros import pub_estop
from functools import partial


# pdb.set_trace()

class CommandGUI:
    '''
    Place where the GUI code is help for sending commands to robots and displaying their information
    '''
    def __init__(self, master, bot_labels, gui_app):
        self.master = master
        master.title("COMMAND CENTER")

        self.num_bots = len(bot_labels) #number of robots
        self.bot_labels = bot_labels #names to be displayed. also corresponds to topic name
        self.bot_status_types = ['Battery', 'Mobility', 'Comms'] #the standard types of status updates displayed for each robot


        #setup the various sub-windows
        self.setupCommandFrame(master)
        self.setupStatusFrame(master)

    
    
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
            self.estop_buts[i].append(Button(self.command_tabs[i], text="Hard e-stop", command = partial(pub_estop,3, self.estop_pubs[i])))
            self.estop_buts[i].append(Button(self.command_tabs[i], text="Soft e-stop", command = partial(pub_estop,3, self.estop_pubs[i]))) 
            self.estop_buts[i].append(Button(self.command_tabs[i], text="Pause", command = partial(pub_estop,3, self.estop_pubs[i]))) 
            self.estop_buts[i].append(Button(self.command_tabs[i], text="Resume", command = partial(pub_estop,3, self.estop_pubs[i]))) 
 

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
                self.bot_status_labels[i].append(Label(robot_frame, text = self.bot_status_types[j]))
                self.bot_status_labels[i].append(Label(robot_frame, text = self.bot_status_types[j]))

            for j in range(len(self.bot_status_labels[i])):
                self.bot_status_labels[i][j].grid()

            robot_frame.grid(row = 1, column = i)
            self.bot_status_frames.append(robot_frame)



>>>>>>> e87ea7bd5198f5eab393cf1238c35683d72689aa


