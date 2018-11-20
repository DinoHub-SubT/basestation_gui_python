#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import Int32

global ser

def callback(msg):
	ser.write(str(msg.data)+'\n')     # write a string

def estop_to_serial():

	# Open serial port
	global ser
	ser = serial.Serial('/dev/ttyUSB0')  # open serial port -- running "dmesg | grep tty" helps with this...
	print(ser.name)         # check which port was really used

	# Setup ROS stuff
	rospy.init_node('estop_to_serial', anonymous=True)
	rospy.Subscriber("/e_stop", Int32, callback)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		rate.sleep()

	# Close port
	ser.close()
	

if __name__ == '__main__':
    try:
        estop_to_serial()
    except rospy.ROSInterruptException:
        pass