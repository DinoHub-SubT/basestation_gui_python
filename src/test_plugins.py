#!/usr/bin/python

# license removed for brevity
import rospy
from basestation_gui_python.msg import GuiMessage
import time

def talker():
    pub = rospy.Publisher('/gui_message_print', GuiMessage, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.5) # 10hz
    msg = GuiMessage()

    while not rospy.is_shutdown():
        msg.data = "hello"+str(time.time())
        # msg.color.r = 255  
        # msg.color.g = 0  
        # msg.color.b = 0  
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass