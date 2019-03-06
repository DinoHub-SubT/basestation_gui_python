#!/usr/bin/python

'''
File which handles the backend of the GUI (storing states, incoming artifacts, 
artifact proposal history, periodic saving of the gui state, etc. )
Contact: Bob DeBortoli (debortor@oregonstate.edu)
'''
import rospy
from basestation_gui_python.msg import RadioMsg
import pdb

# pdb.set_trace()


class GuiEngine:
    '''
    Class that is backend for the gui
    '''
    def __init__(self):
        self.queued_artifacts = []
        self.submitted_artifacts = []


        #if we're simulating incoming artifact detections
        self.simulating_artifact_detections = rospy.get_param("/simulating_artifact_detections")

        #start subscriber to listen for incoming artifact detections
        rospy.Subscriber('/incoming_artifact', RadioMsg, self.addIncomingArtifact)



    def addIncomingArtifact(self, msg):
        '''
        Add an incoming artifact to the queue
        '''

        #convert the detection into a gui artifact type, which includes more data

        #add the artifact to the list of queued objects

        #graphically add it to the queue

        pass





class Artifact:
    '''
    Class to handle artifact as an object in the gui
    '''
    def __init__(self):
        self.category = ""
        self.pos = [-1, -1, -1]
        self.id = -1
        self.time_from_robot = -1 #time the detection has come in from the robot
        self.time_to_darpa = -1 #time submitted to darpa



