#!/usr/bin/env python
import Tkinter as tk
    


# license removed for brevity
import rospy
from std_msgs.msg import Int32
from mavros_msgs.msg import Mavlink #sudo apt-get install ros-kinetic-mavros


global root
global pub
global button1
global button2
global button3
global button4
global state
global button_bg_colour

def talker():

    global pub
    global pub2
    pub = rospy.Publisher('/mavlink/from', Mavlink, queue_size=10)
    # pub = rospy.Publisher('/e_stop', Int32, queue_size=10)
    pub2 = rospy.Publisher('/e_stop', Int32, queue_size=10)
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
    if state==1:
        button1.config(bg='red')
        button1.config(activebackground='red')
    if state==2:
        button2.config(bg='red')
        button2.config(activebackground='red')
    if state==3:
        button3.config(bg='orange')
        button3.config(activebackground='orange')
    if state==4:
        button4.config(bg='green')
        button4.config(activebackground='green')   

def buttons_process(number):
    print(number)
    msg = Mavlink()
    msg_int = Int32()
    msg_int.data = number
    msg.payload64 = [number]
    pub.publish(msg)
    # pub.publish(msg_int)
    pub2.publish(msg_int)
    global state
    state = number
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
    buttons_process(3)
    window_are_you_sure.destroy()

def close_window_are_you_sure():
    window_are_you_sure.destroy()

def button_callback2():
    buttons_process(2)

def button_callback3():
    buttons_process(1)

def button_callback4():
    buttons_process(0)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


