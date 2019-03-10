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

    rate = rospy.Rate(5.2) #0.3 rate in hz

    msg = RadioMsg()
    msg.message_type =  RadioMsg.MESSAGE_TYPE_ARTIFACT_REPORT
    
    
    


    while not rospy.is_shutdown():
        msg.artifact_report_id =  random.randint(0,1000)
        msg.artifact_type =  random.sample(artifact_types,1)[0]
        msg.artifact_robot_id = random.randint(0,1)
        msg.artifact_x =  random.random()*2000
        msg.artifact_y =  random.random()*2000
        msg.artifact_z =  random.random()*2000

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass