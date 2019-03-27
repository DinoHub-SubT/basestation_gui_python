#!/usr/bin/python

'''
File which handles the backend of the GUI (storing states, incoming artifacts, 
artifact proposal history, periodic saving of the gui state, etc. )
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebation or Matt).
'''
import rospy
from basestation_gui_python.msg import RadioMsg, FakeWifiDetection
import pdb
import time
import copy
import rospkg
import csv
import datetime
from cv_bridge import CvBridge
import os
import cv2

# pdb.set_trace()


class GuiEngine:
    '''
    Class that is backend for the gui
    '''
    def __init__(self, gui):
        self.queued_artifacts = [] #saved for later
        self.submitted_artifacts = [] #submitted to darpa
        self.all_artifacts = [] #every artifact ever detected, submitted, deleted, etc.
        self.br = CvBridge() #bridge from opencv to ros image messages


        #if we're simulating incoming artifact detections
        self.simulating_artifact_detections = rospy.get_param("/simulating_artifact_detections")

        if(self.simulating_artifact_detections):

            #start subscriber to listen for incoming artifact detections
            rospy.Subscriber('/fake_artifact_detections', RadioMsg, self.addRadioMsgDetection)

            #start subscriber to listen for incoming artifact detection images
            rospy.Subscriber('/fake_artifact_imgs', FakeWifiDetection, self.addWifiMsgDetection)



        else:
           #here is where we would put the subscriber for real detections
           rospy.Subscriber('/real_artifact_detections', RadioMsg, self.addRadioMsgDetection)
           rospy.Subscriber('/ugv1/real_artifact_detections', RadioMsg, self.addRadioMsgDetection)
           rospy.Subscriber('/uav1/real_artifact_detections', RadioMsg, self.addRadioMsgDetection)
            
           rospy.Subscriber('/real_artifact_imgs', FakeWifiDetection, self.addWifiMsgDetection)
           rospy.Subscriber('/ugv1/real_artifact_imgs', FakeWifiDetection, self.addWifiMsgDetection)
           rospy.Subscriber('/uav1/real_artifact_imgs', FakeWifiDetection, self.addWifiMsgDetection)

        self.gui = gui

        self.initLogFolder()

        self.duplicate_count = 0
        

    def processIncomingMsg(self, msg):
        '''
        !!! DEPRICATED !!!  Process an incoming RadioMsg !!! DEPRICATED !!!
        '''
        pass
        # if (msg.message_type==RadioMsg.MESSAGE_TYPE_ARTIFACT_REPORT):
        #     self.addRadioMsgDetection(msg)
        # else:
        #     print "We are getting messages other than artifact detections and we don't know what to do with it"


    def addWifiMsgDetection(self, msg):
        '''
        Add an incoming artifact to the queue, or update an existing one
        '''

        if (msg.artifact_type == FakeWifiDetection.ARTIFACT_REMOVE): #we're removing an artifact not adding one

            self.removeArtifactFromGui(msg)

        else: #add an artifact

            #convert the message to an opencv image
            imgs = []
            img_stamps = []

            if (len(msg.imgs) > 0):
                
                for img in msg.imgs:
                    cv_image = self.br.imgmsg_to_cv2(img)
                    cv2.putText(cv_image, "Timestamp: %f" %
                        img.header.stamp.to_sec(),
                            (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                            (255, 255, 255), 1)
                    imgs.append(cv_image)
                    img_stamps.append(img.header.stamp)


            #check if the artifact already exists
            already_object = False

            msg_unique_id = str(msg.artifact_robot_id)+'/'+str(msg.artifact_report_id)+'/'+str(msg.artifact_stamp.secs)

            for artifact in self.all_artifacts:
                if msg_unique_id == artifact.unique_id:

                    #update the current information if it exists!
                    if ( len(imgs) > 0):

                        for i, img in enumerate(imgs):
                            artifact.imgs.append(img)
                            artifact.img_stamps.append(img_stamps[i])
                            sorted_tuple = sorted(zip(artifact.img_stamps,
                                artifact.imgs), key=lambda t: t[0],
                                reverse=True)
                            artifact.img_stamps = [t[0] for t in sorted_tuple]
                            artifact.imgs = [t[1] for t in sorted_tuple]
                            print "Sorted stamps", artifact.img_stamps

                    if (msg.artifact_x != 0) and (msg.artifact_y != 0) and (msg.artifact_z != 0):
                        artifact.pos[0] = msg.artifact_x
                        artifact.pos[1] = msg.artifact_y
                        artifact.pos[2] = msg.artifact_z

                    if(msg.artifact_type != ''):
                        artifact.category = self.gui.label_to_cat_dict[msg.artifact_type]


                    #update the GUI
                    self.gui.updateArtifactInQueue(artifact)
                    

                    already_object = True


            #else, make a new artifact
            if (not already_object):
                artifact = Artifact(msg.artifact_stamp.secs, self.gui.label_to_cat_dict[msg.artifact_type],  [msg.artifact_x, msg.artifact_y, msg.artifact_z], \
                                    msg.artifact_robot_id,
                                    msg.artifact_report_id, imgs, img_stamps)


                #add the artifact to the list of queued objects and to the all_artifacts list
                self.queued_artifacts.append(artifact)
                self.all_artifacts.append(artifact)

                #call a function to graphically add it to the queue
                self.gui.sendToQueue(artifact)

                #add updated info to the csv
                self.savePeriodically(self.gui)

    def duplicateArtifact(self, artifact):
        '''
        Generate another artifact with the same properties as the one passed in
        '''

        #find a unique negative id
        artifact_id = 0

        negative_id_list = []
        for art in self.all_artifacts:
            if (art.artifact_report_id < 0):
                negative_id_list.append(art.artifact_report_id)

        artifact_id = (len(negative_id_list) + 1) * -1

        if (artifact_id < 0):

            #determine the robot_id
            if (artifact.source_robot == 0):
                art_source_id = -1
            elif (artifact.source_robot == 1):
                art_source_id = -2
            else:
                art_source_id = None
                self.gui.printMessage('Could not determine source id for this dulication, so artifact not duplicated')

            if (art_source_id != None):
                #generate the artifact object

                artifact = Artifact(copy.deepcopy(artifact.original_timestamp), copy.deepcopy(artifact.category), \
                                    copy.deepcopy(artifact.pos), art_source_id,
                                    artifact_id, copy.deepcopy(artifact.imgs),
                                    copy.deepcopy(artifact.img_stamps))

                #add the artifact to the list of queued objects and to the all_artifacts list
                self.queued_artifacts.append(artifact)
                self.all_artifacts.append(artifact)

                #call a function to graphically add it to the queue
                self.gui.sendToQueue(artifact)

                #add updated info to the csv
                self.savePeriodically(self.gui)



    def addArtifactManually(self):
        '''
        Function which allows the BSM to manually insert an artifact
        at any point
        '''

        #find a unique negative id
        artifact_id = 0

        negative_id_list = []
        for artifact in self.all_artifacts:
            if (artifact.artifact_report_id < 0):
                negative_id_list.append(artifact.artifact_report_id)

        artifact_id = (len(negative_id_list) + 1) * -1



        if (artifact_id < 0):
            #generate the artifact object
            artifact = Artifact(1, self.gui.ros_gui_bridge.artifact_categories[0], [-0.1, -0.1, -0.1], \
                                    -1, artifact_id, [])

            #add the artifact to the list of queued objects and to the all_artifacts list
            self.queued_artifacts.append(artifact)
            self.all_artifacts.append(artifact)

            #call a function to graphically add it to the queue
            self.gui.sendToQueue(artifact)

            #add updated info to the csv
            self.savePeriodically(self.gui)

    def removeArtifactFromGui(self, msg):
        '''
        If an artifact has type removal, remove it from the gui and engine
        '''

        #find the artifact being referneced
        artifact = None

        for art in self.all_artifacts:

            msg_unique_id = str(msg.artifact_robot_id)+'/'+str(msg.artifact_report_id)+'/'+str(msg.artifact_stamp.secs)
            
            if msg_unique_id == art.unique_id:
                artifact = art
                break

        #ensure we dont delete an artifact when we're viewing it
        if (self.gui.displayed_artifact != None and artifact != None):
            if (self.gui.displayed_artifact.unique_id == artifact.unique_id):
                self.gui.printMessage('Artifact being viewed has been deleted')
                self.gui.alertImgAboutRemoval()
                self.gui.removeQueueArtifact(artifact)

        else:
            if(artifact == None):
                self.gui.printMessage('Did not delete artifact, could not find it')
            else:
                self.gui.removeQueueArtifact(artifact)


    def addRadioMsgDetection(self, msg):
        '''
        Add an incoming artifact to the queue
        '''

        if (msg.artifact_type == RadioMsg.ARTIFACT_REMOVE): #we're removing an artifact not adding one

            self.removeArtifactFromGui(msg)

        else: #add an artifact

            #make sure we don't have a duplicate artifact
            already_object = False

            for artifact in self.all_artifacts:

                msg_unique_id = str(msg.artifact_robot_id)+'/'+str(msg.artifact_report_id)+'/'+str(msg.artifact_stamp.secs)
                
                if msg_unique_id == artifact.unique_id:
                    
                    #update the current information if it exists!
                    if (msg.artifact_x != 0) and (msg.artifact_y != 0) and (msg.artifact_z != 0):
                        artifact.pos[0] = msg.artifact_x
                        artifact.pos[1] = msg.artifact_y
                        artifact.pos[2] = msg.artifact_z

                    if(msg.artifact_type != ''):
                        artifact.category = self.gui.label_to_cat_dict[msg.artifact_type]

                    #update the GUI
                    self.gui.updateArtifactInQueue(artifact)
                    
                    already_object = True

            if (not already_object):


                #convert the detection into a gui artifact type, which includes more data
                artifact = Artifact(msg.artifact_stamp.secs, self.gui.label_to_cat_dict[msg.artifact_type],  [msg.artifact_x, msg.artifact_y, msg.artifact_z], \
                                    msg.artifact_robot_id, msg.artifact_report_id, [])


                #add the artifact to the list of queued objects and to the all_artifacts list
                self.queued_artifacts.append(artifact)
                self.all_artifacts.append(artifact)

                #call a function to graphically add it to the queue
                self.gui.sendToQueue(artifact)

                #add updated info to the csv
                self.savePeriodically(self.gui)

            print "Successfully generated artifact", len(self.all_artifacts), artifact.pos,  artifact.artifact_report_id, artifact.source_robot, artifact.original_timestamp


    def initLogFolder(self):
        '''
        Make the log folder to save gui information to
        '''

        rospack = rospkg.RosPack()

        #define the file structure
        self.log_folder = rospack.get_path('basestation_gui_python')+'/custom_logs/log_'+datetime.datetime.now().strftime("%m-%d-%y-%H-%M-%S")+'/'
        self.log_img_folder = self.log_folder+'imgs/'

        #make the folders
        if not os.path.exists(self.log_folder):
            os.makedirs(self.log_folder)
            os.makedirs(self.log_img_folder)


        self.log_filename = self.log_folder+'gui_state_log.csv'

        with open(self.log_filename, 'w') as writeFile:
            writer = csv.writer(writeFile)
            writer.writerow(['Time Standard', 'Artifacts', 'Vehicle E-stop State', 'Score/Remaining Reports/Run Clock'])



    def savePeriodically(self, gui):
        '''
        Saves the information in the gui in a re-loadable fromat, should the gui crash
        '''

        #artifact save format: [category, pos, orig_pos, source_robot, report_id, time_from_robot, time_to_darpa, unread_priority, darpa_response, image filenames]
        artifact_str = ''
        for artifact in self.all_artifacts:

            if (artifact.darpa_response == None):
                darpa_text = ''
            else:
                darpa_text = artifact.darpa_response.replace('\n','')

            artifact_str   +=    artifact.category+'|'+ str(artifact.pos[0])+','+str(artifact.pos[0])+','+str(artifact.pos[0])+'|'+ \
                                 str(artifact.orig_pos[0])+','+str(artifact.orig_pos[1])+','+str(artifact.orig_pos[2])+\
                                 '|'+ str(artifact.source_robot)+'|'+\
                                 str(artifact.artifact_report_id)+'|'+ str(artifact.time_from_robot)+'|'+ str(artifact.time_to_darpa)+'|'+\
                                 str(artifact.unread)+ '|'+artifact.priority+'|'+ darpa_text + '|'+ artifact.unique_id+ '|'+\
                                 str(artifact.original_timestamp) +'|'+'//'

        #save the vehicle statuses
        vehicle_state_str = ''
        for robot_buttons in self.gui.control_buttons:
            for i, button in enumerate(robot_buttons):
                if (button.isChecked()) and  (button.text() in self.gui.ros_gui_bridge.estop_commands): #if the estop button is checked
                    vehicle_state_str+=str(button.text())
            vehicle_state_str+='|'


        #save the run info
        info_str = ''
        if (self.gui.darpa_gui_bridge.darpa_status_update != {}): #we have actually received a message

            if (self.gui.darpa_gui_bridge.darpa_status_update['score'] != None):
                info_str += str(self.gui.darpa_gui_bridge.darpa_status_update['score'])

            info_str += '|'

            if (self.gui.darpa_gui_bridge.darpa_status_update['remaining_reports'] != None):
                info_str += str(self.gui.darpa_gui_bridge.darpa_status_update['remaining_reports'])

            info_str += '|'

            if (self.gui.darpa_gui_bridge.darpa_status_update['run_clock'] != None):
                info_str += str(self.gui.darpa_gui_bridge.darpa_status_update['run_clock'])


        with open(self.log_filename, 'a') as writeFile:
            writer = csv.writer(writeFile)
            writer.writerow([time.time(), artifact_str, vehicle_state_str, info_str]) 


        #save the images
        for artifact in self.all_artifacts:
            for i, img in enumerate(artifact.imgs):
                fname = self.log_img_folder+ artifact.category + '_' + artifact.unique_id.replace('/','_')+'.png'
                img = cv2.resize(img,(300, 300))
                cv2.imwrite(fname, img)





class Artifact:
    '''
    Class to handle artifact as an object in the gui
    '''
    def __init__(self, original_timestamp=1, category=-1, position="",
            source_robot_id="", artifact_report_id="", imgs=None, 
            img_stamps=None):
        
        self.category = category
        self.pos = position
        self.orig_pos = copy.deepcopy(position)
        self.source_robot = source_robot_id
        self.artifact_report_id = artifact_report_id
        self.time_from_robot = -1 #time the detection has come in from the robot. TODO: change to be something different?
        self.time_to_darpa = -1 #time submitted to darpa
        self.unread = True
        self.priority = 'Med'
        self.darpa_response = ''
        self.imgs = imgs if imgs is not None else []
        self.img_stamps = img_stamps if img_stamps is not None else []
        self.original_timestamp = original_timestamp
        self.unique_id = str(source_robot_id)+'/'+str(artifact_report_id)+'/'+str(original_timestamp)




