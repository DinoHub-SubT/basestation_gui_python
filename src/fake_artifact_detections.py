#!/usr/bin/python

'''
File which publishes fake artifact detections
Contact: Bob DeBortoli (debortor@oregonstate.edu)
'''
import rospy
from basestation_gui_python.msg import RadioMsg
import pdb
import random

# pdb.set_trace()


def talker():
    pub = rospy.Publisher('/fake_artifact_detections', RadioMsg, queue_size=10)
    rospy.init_node('fake_artifact_node', anonymous=True)

    rate = rospy.Rate(0.5) # 10hz

    msg = RadioMsg()
    msg.message_type =  1
    msg.artifact_x =  1.
    msg.artifact_y =  2.
    msg.artifact_z =  1.
    msg.artifact_type =  'Fire Extinguisher'
    msg.artifact_robot_id = 0
    


    while not rospy.is_shutdown():
        msg.artifact_report_id =  random.randint(0,100)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass