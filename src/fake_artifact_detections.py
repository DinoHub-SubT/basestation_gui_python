#!/usr/bin/python

'''
File which publishes fake artifact detections
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebation or Matt).
'''
import rospy
from basestation_gui_python.msg import RadioMsg, FakeWifiDetection
import pdb
import random
import time
from nav_msgs.msg import Odometry

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import rospkg

# pdb.set_trace()



def talker():
    pub = rospy.Publisher('/fake_artifact_detections', RadioMsg, queue_size=10)
    img_pub = rospy.Publisher('/fake_artifact_imgs', FakeWifiDetection, queue_size=10)
    
    rospy.init_node('fake_artifact_node', anonymous=True)
    artifact_types = ['Human', 'Fire extinguisher', 'Phone', 'Backpack', 'Drill']

    total_num_to_pub = 10#1000
    num_pubbed = 0    

    #add stuff for testing ji button pipeline
    # ji_pub = rospy.Publisher('/integrated_to_map', Odometry, queue_size=10)
    
    rate = rospy.Rate(0.2) #rate in hz

    time.sleep(2.) #necessary because launch order is random

    msg = RadioMsg()
    msg.message_type =  RadioMsg.MESSAGE_TYPE_ARTIFACT_REPORT

    while not rospy.is_shutdown() and num_pubbed < total_num_to_pub:
        msg.artifact_report_id =  random.randint(0,9999)
        msg.artifact_type =  random.sample(artifact_types,1)[0]
        msg.artifact_robot_id = random.randint(0,1)
        msg.artifact_x =  random.random()*5.
        msg.artifact_y =  random.random()*5.
        msg.artifact_z =  random.random()*5.

        pub.publish(msg)

        if (num_pubbed == 0):
            initial_report_id = msg.artifact_report_id
            initial_type = msg.artifact_type
            initial_robot_id = msg.artifact_robot_id
            initial_pos = [msg.artifact_x, msg.artifact_y, msg.artifact_z]

            # print initial_robot_id, initial_report_id, initial_type

        if(num_pubbed == total_num_to_pub - 1):

            # print "here"
            # print initial_robot_id, initial_report_id

            img_msg = getFakeWifiMsg(artifact_report_id = initial_report_id, artifact_type = 'Drill', \
                                     artifact_robot_id = initial_robot_id, artifact_pos = [initial_pos[0]+1, initial_pos[1], initial_pos[2]+1])
                                    # (artifact_report_id = msg.artifact_report_id, artifact_type = msg.artifact_type, \
                                    #  artifact_robot_id = msg.artifact_robot_id, artifact_pos = [msg.artifact_x, msg.artifact_y, msg.artifact_z])
                                    #(artifact_report_id = None, artifact_type = None, artifact_robot_id = None, artifact_pos = None)

            img_pub.publish(img_msg)

        rate.sleep()

        num_pubbed+=1

def getJiFakePose():
    ji_msg = Odometry()

    ji_msg.pose.pose.position.x = 1
    ji_msg.pose.pose.position.y = 2
    ji_msg.pose.pose.position.z = 3.14

    return ji_msg

def getFakeWifiMsg(artifact_report_id, artifact_type , artifact_robot_id , artifact_pos ):
    rospack = rospkg.RosPack()
    if (artifact_type == 'Fire extinguisher'):
        image_filename = rospack.get_path('basestation_gui_python')+'/fake_artifact_imgs/test_img.jpg'
    elif (artifact_type == 'Human'):
        image_filename = rospack.get_path('basestation_gui_python')+'/fake_artifact_imgs/human.png'
    elif (artifact_type == 'Drill'):
        image_filename = rospack.get_path('basestation_gui_python')+'/fake_artifact_imgs/drill.jpg'
    elif (artifact_type == 'Backpack'):
        image_filename = rospack.get_path('basestation_gui_python')+'/fake_artifact_imgs/backpack.png'
    elif (artifact_type == 'Phone'):
        image_filename = rospack.get_path('basestation_gui_python')+'/fake_artifact_imgs/cell_phone.png'
    else:
        image_filename = rospack.get_path('basestation_gui_python')+'/fake_artifact_imgs/cell_phone.png'

    img = cv2.imread(image_filename)

    br = CvBridge()
    img = br.cv2_to_imgmsg(img)

    if (artifact_report_id == None): #we need to generate some fake data
        msg = None

    else:
        msg = FakeWifiDetection()

        msg.img = img 
        msg.artifact_robot_id = artifact_robot_id
        msg.artifact_report_id = artifact_report_id
        msg.artifact_type = artifact_type
        msg.artifact_x = artifact_pos[0]
        msg.artifact_y = artifact_pos[1]
        msg.artifact_z = artifact_pos[2]
    

    return msg



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass