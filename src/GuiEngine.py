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



        # else:
        #     #here is where we would put the subscriber for real detections
        #     rospy.Subscriber('/ugv1/real_artifact_detections', RadioMsg, self.addRadioMsgDetection)
        #     rospy.Subscriber('/uav1/real_artifact_detections', RadioMsg, self.addRadioMsgDetection)

        self.gui = gui

        self.initLogFile()

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


        #convert the message to an opencv image
        if (msg.img.data != ''):
            img = self.br.imgmsg_to_cv2(msg.img)
        else:
            img = np.array([])


        #check if the artifact already exists
        already_object = False

        for artifact in self.all_artifacts:
            if int(msg.artifact_robot_id) == int(artifact.source_robot) and \
                int(msg.artifact_report_id) == int(artifact.artifact_report_id):

                #update the current information if it exists!
                if (len(img.shape)>1):
                    artifact.imgs.append(img)
                else:
                    print "image data not updated"

                if (msg.artifact_x != 0) and (msg.artifact_y != 0) and (msg.artifact_z != 0):
                    artifact.pos[0] = msg.artifact_x
                    artifact.pos[1] = msg.artifact_y
                    artifact.pos[2] = msg.artifact_z

                if(msg.artifact_type != ''):
                    artifact.category = msg.artifact_type


                #update the GUI
                self.gui.updateArtifactInQueue(artifact)
                

                already_object = True


        #else, make a new artifact
        if (not already_object):

            if(len(img.shape)>1):
                artifact = Artifact(msg.artifact_type, [msg.artifact_x, msg.artifact_y, msg.artifact_z], \
                                        msg.artifact_robot_id, msg.artifact_report_id, [img])

            else:
                artifact = Artifact(msg.artifact_type, [msg.artifact_x, msg.artifact_y, msg.artifact_z], \
                                        msg.artifact_robot_id, msg.artifact_report_id, [])


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

            #generate the artifact object
            artifact = Artifact(copy.deepcopy(artifact.category), copy.deepcopy(artifact.pos), \
                                    -1, artifact_id, [])

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
            artifact = Artifact(self.gui.ros_gui_bridge.artifact_categories[0], [-0.1, -0.1, -0.1], \
                                    -1, artifact_id, [])

            #add the artifact to the list of queued objects and to the all_artifacts list
            self.queued_artifacts.append(artifact)
            self.all_artifacts.append(artifact)

            #call a function to graphically add it to the queue
            self.gui.sendToQueue(artifact)

            #add updated info to the csv
            self.savePeriodically(self.gui)


    def addRadioMsgDetection(self, msg):
        '''
        Add an incoming artifact to the queue
        '''

        if (msg.artifact_type == FakeWifiDetection.ARTIFACT_REMOVE): #we're removing an artifact not adding one

            #find the artifact being referneced
            artifact = None

            for art in self.all_artifacts:

                msg_unique_id = str(msg.artifact_robot_id)+'/'+str(msg.artifact_report_id)+'/'+str(msg.artifact_stamp.secs)
                
                if msg_unique_id == art.unique_id:
                    artifact = art
                    break


            if(artifact == None):
                self.gui.printMessage('Did not delete artifact, could not find it')
            
            else:
                self.gui.removeQueueArtifact(artifact)


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


    def initLogFile(self):
        '''
        Make the log file to save gui information to
        '''

        rospack = rospkg.RosPack()
        self.log_filename = rospack.get_path('basestation_gui_python')+'/custom_logs/log_'+datetime.datetime.now().strftime("%m-%d-%y-%H-%M-%S")+'.csv'

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
                                 str(artifact.unread)+ '|'+artifact.priority+'|'+ darpa_text + '|'+ '//'

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





class Artifact:
    '''
    Class to handle artifact as an object in the gui
    '''
    def __init__(self, original_timestamp, category=-1, position="", source_robot_id="", artifact_report_id="", imgs = []):
        
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
        self.imgs = imgs
        self.original_timestamp = original_timestamp
        self.unique_id = str(source_robot_id)+'/'+str(artifact_report_id)+'/'+str(original_timestamp)




