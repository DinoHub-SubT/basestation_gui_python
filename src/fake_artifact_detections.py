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
    artifact_types = ['human', 'extinguisher', 'phone', 'backpack', 'drill']

    rate = rospy.Rate(0.1) #0.3 rate in hz

    msg = RadioMsg()
    msg.message_type =  RadioMsg.MESSAGE_TYPE_ARTIFACT_REPORT
    msg.artifact_x =  1.
    msg.artifact_y =  2.
    msg.artifact_z =  1.
    
    


    while not rospy.is_shutdown():
        msg.artifact_report_id =  random.randint(0,100)
        msg.artifact_type =  artifact_types[random.randint(0,4)]
        msg.artifact_robot_id = random.randint(0,1)

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass