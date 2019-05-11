#!/usr/bin/python

# license removed for brevity
import rospy
from basestation_gui_python.msg import GuiMessage
import time
import random

def talker():
    pub = rospy.Publisher('/gui_message_print', GuiMessage, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.1) # 10hz
    # msg = GuiMessage()

    while not rospy.is_shutdown():
        msg = GuiMessage()
        msg.data = "hello"+str(time.time())
        msg.color.r = random.random()*126. + 126.  
        msg.color.g = random.random()*126. + 126.  
        msg.color.b = random.random()*126. + 126.  
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass