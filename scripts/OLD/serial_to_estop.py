#!/usr/bin/env python


# !!!
# possibly need to run this first:
# sudo chmod 666 /dev/ttyUSB0
# TODO look for a more permanent solution...
# !!!

import serial
import rospy
from std_msgs.msg import Int32

global ser

def serial_to_estop():

	# Open serial port
	global ser
	ser = serial.Serial('/dev/ttyUSB0', timeout=1)  # open serial port -- running "dmesg | grep tty" helps with this...
	print(ser.name)         # check which port was really used

	# Setup ROS stuff
	rospy.init_node('serial_to_estop', anonymous=True)
	pub = rospy.Publisher("/e_stop", Int32, queue_size=10)

	# Listener loop
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		line = ser.readline()
		print line
		# extract number and publish
		msg = Int32()
		try:
			msg.data = int(line)
			pub.publish(msg)
		except ValueError:
			print ''
		rate.sleep()

	while not rospy.is_shutdown():
		rate.sleep()

	# Close port
	ser.close()
	

if __name__ == '__main__':
    try:
        serial_to_estop()
    except rospy.ROSInterruptException:
        pass
