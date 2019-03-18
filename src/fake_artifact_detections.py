#!/usr/bin/python

'''
File which publishes fake artifact detections
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebation or Matt).
'''
import rospy
from basestation_gui_python.msg import RadioMsg
import pdb
import random
import time
from nav_msgs.msg import Odometry

# pdb.set_trace()

def getJiFakePose():
    ji_msg = Odometry()

    ji_msg.pose.pose.position.x = 1
    ji_msg.pose.pose.position.y = 2
    ji_msg.pose.pose.position.z = 3.14

    return ji_msg


def talker():
    pub = rospy.Publisher('/fake_artifact_detections', RadioMsg, queue_size=10)
    rospy.init_node('fake_artifact_node', anonymous=True)
    artifact_types = ['Human', 'Fire extinguisher', 'Phone', 'Backpack', 'Drill']

    rate = rospy.Rate(0.2) #rate in hz

    msg = RadioMsg()
    msg.message_type =  RadioMsg.MESSAGE_TYPE_ARTIFACT_REPORT

    total_num_to_pub = 10#1000
    num_pubbed = 0    

    time.sleep(2.)
    print "done"

    #add stuff for testing ji button pipeline
    ji_pub = rospy.Publisher('/integrated_to_map', Odometry, queue_size=10)


    while not rospy.is_shutdown() and num_pubbed < total_num_to_pub:
        msg.artifact_report_id =  random.randint(0,9999)
        msg.artifact_type =  random.sample(artifact_types,1)[0]
        msg.artifact_robot_id = random.randint(0,1)
        msg.artifact_x =  random.random()*5.
        msg.artifact_y =  random.random()*5.
        msg.artifact_z =  random.random()*5.

        # print "pubbed: ", num_pubbed

        # ji_pub.publish(getJiFakePose())
        
        pub.publish(msg)
        rate.sleep()

        num_pubbed+=1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass