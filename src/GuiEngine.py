#!/usr/bin/python

'''
File which handles the backend of the GUI (storing states, incoming artifacts, 
artifact proposal history, periodic saving of the gui state, etc. )
Contact: Bob DeBortoli (debortor@oregonstate.edu)
'''
import rospy
from basestation_gui_python.msg import RadioMsg
import pdb
import time
import Gui

# pdb.set_trace()


class GuiEngine:
    '''
    Class that is backend for the gui
    '''
    def __init__(self, gui):
        self.queued_artifacts = [] #saved for later
        self.submitted_artifacts = [] #submitted to darpa
        self.all_artifacts = [] #every artifact ever detected, submitted, deleted, etc.


        #if we're simulating incoming artifact detections
        self.simulating_artifact_detections = rospy.get_param("/simulating_artifact_detections")

        if(self.simulating_artifact_detections):

            #start subscriber to listen for incoming artifact detections
            rospy.Subscriber('/fake_artifact_detections', RadioMsg, self.addIncomingArtifact)

        else:
            #here is where we would put the scubscriber for real detections
            rospy.Subscriber('/real_artifact_detections', RadioMsg, self.addIncomingArtifact)

        self.gui = gui

    def processIncomingMsg(self, msg):
        '''
        Process an incoming RadioMsg
        '''
        if (msg.message_type==RadioMsg.MESSAGE_TYPE_ARTIFACT_REPORT):
            self.addIncomingArtifact(msg)
        else:
            print "We are getting messages other than artifact detections and we don't know what to do with it"


    def addIncomingArtifact(self, msg):
        '''
        Add an incoming artifact to the queue
        '''

        #convert the detection into a gui artifact type, which includes more data
        artifact = Artifact(msg.artifact_type, [msg.artifact_x, msg.artifact_y, msg.artifact_z], \
                            msg.artifact_robot_id, msg.artifact_report_id)

        #add the artifact to the list of queued objects and to the all_artifacts list
        self.queued_artifacts.append(artifact)
        self.all_artifacts.append(artifact)

        #call a function to graphically add it to the queue
        self.gui.sendToQueue(artifact)






class Artifact:
    '''
    Class to handle artifact as an object in the gui
    '''
    def __init__(self, category, position, source_robot_id, artifact_report_id):
        self.category = category
        self.pos = position
        self.source_robot = source_robot_id
        self.artifact_report_id = artifact_report_id
        self.time_from_robot = -1 #time the detection has come in from the robot. TODO: change to be something different?
        self.time_to_darpa = -1 #time submitted to darpa
        self.unread = True
        self.priority = 'Med'



