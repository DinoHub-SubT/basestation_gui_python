#!/usr/bin/python

'''
File which publishes fake artifact detections
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebation or Matt).
'''
import rospy
from basestation_gui_python.msg import RadioMsg, FakeWifiDetection
from std_msgs.msg import String
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
    message_pub = rospy.Publisher('/gui_message_listener', String, queue_size=10)
    
    rospy.init_node('fake_artifact_node', anonymous=True)
    artifact_types = [4, 3, 5, 1, 2]

    total_num_to_pub = 20#1000
    num_pubbed = 0    

    #add stuff for testing ji button pipeline
    # ji_pub = rospy.Publisher('/integrated_to_map', Odometry, queue_size=10)
    
    rate = rospy.Rate(0.2) #rate in hz

    time.sleep(2.) #necessary because launch order is random

    msg = RadioMsg()
    msg.message_type =  RadioMsg.MESSAGE_TYPE_ARTIFACT_REPORT

    published_list = []

    while not rospy.is_shutdown() and num_pubbed < total_num_to_pub:
        msg.artifact_report_id =  random.randint(0,9999)
        msg.artifact_type =  random.sample(artifact_types,1)[0]
        msg.artifact_robot_id = random.randint(0,1)
        msg.artifact_x =  random.random()*5.
        msg.artifact_y =  random.random()*5.
        msg.artifact_z =  random.random()*5.
        msg.artifact_stamp.secs = time.time() 

        if (num_pubbed < total_num_to_pub * 0.3):
            published_list.append([msg.artifact_robot_id, msg.artifact_report_id, msg.artifact_stamp.secs])
            pub.publish(msg)

        else: #update some artifacts

            rand_ind = random.randint(0,len(published_list)-1)
            robot_id = published_list[rand_ind][0]
            report_id = published_list[rand_ind][1]
            timestamp = published_list[rand_ind][2]

            if (random.random()>0.5): #update by radio message
                msg.artifact_report_id =  report_id
                if (random.random() < 0.1):
                    msg.artifact_type = RadioMsg.ARTIFACT_REMOVE
                else:
                    msg.artifact_type =  random.sample(artifact_types,1)[0]
                msg.artifact_robot_id = robot_id
                msg.artifact_x =  random.random()*5.
                msg.artifact_y =  random.random()*5.
                msg.artifact_z =  random.random()*5.
                msg.artifact_stamp.secs = timestamp


                print "radiomsg update to be ",msg.artifact_type, msg.artifact_x, msg.artifact_y, msg.artifact_z
                pub.publish(msg)
                

            else: #update by wifi message

                if (random.random() < 0.1):
                    typ = FakeWifiDetection.ARTIFACT_REMOVE
                else:
                    typ =  random.sample(artifact_types,1)[0]

                img_msg = getFakeWifiMsg(artifact_report_id = report_id , artifact_type = typ, \
                                         artifact_robot_id = robot_id, artifact_pos = [random.random()*5., 0, 1.2], timestamp = timestamp)

                print "wifimsg to be cat",typ, "len of imgs:", len(img_msg.imgs)

                img_pub.publish(img_msg)

        message_pub.publish('Message system testing')

        rate.sleep()

        num_pubbed+=1

def getJiFakePose():
    ji_msg = Odometry()

    ji_msg.pose.pose.position.x = 1
    ji_msg.pose.pose.position.y = 2
    ji_msg.pose.pose.position.z = 3.14

    return ji_msg

def getFakeWifiMsg(artifact_report_id, artifact_type , artifact_robot_id , artifact_pos, timestamp):
    rospack = rospkg.RosPack()

    'survivor', 'fire extinguisher', 'phone', 'backpack', 'drill'
    [4, 3, 5, 1, 2]

    if (artifact_type == 3):
        image_filename = rospack.get_path('basestation_gui_python')+'/fake_artifact_imgs/test_img.jpg'
    elif (artifact_type == 4):
        image_filename = rospack.get_path('basestation_gui_python')+'/fake_artifact_imgs/human.png'
    elif (artifact_type == 2):
        image_filename = rospack.get_path('basestation_gui_python')+'/fake_artifact_imgs/drill.jpg'
    elif (artifact_type == 1):
        image_filename = rospack.get_path('basestation_gui_python')+'/fake_artifact_imgs/backpack.png'
    elif (artifact_type == 5):
        image_filename = rospack.get_path('basestation_gui_python')+'/fake_artifact_imgs/cell_phone.png'
    elif (artifact_type == FakeWifiDetection.ARTIFACT_REMOVE):
        image_filename = rospack.get_path('basestation_gui_python')+'/fake_artifact_imgs/cell_phone.png'

    img = cv2.imread(image_filename)

    br = CvBridge()
    img = br.cv2_to_imgmsg(img)

    if (artifact_report_id == None): #we need to generate some fake data
        msg = None

    else:
        msg = FakeWifiDetection()

        msg.imgs = [img]*random.randint(0,4)
        msg.artifact_robot_id = artifact_robot_id
        msg.artifact_report_id = artifact_report_id
        msg.artifact_type = artifact_type
        msg.artifact_x = artifact_pos[0]
        msg.artifact_y = artifact_pos[1]
        msg.artifact_z = artifact_pos[2]
        msg.artifact_stamp.secs = timestamp
    

    return msg



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass