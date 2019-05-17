#!/usr/bin/python
'''
Functions to handle the artifacts. From detections, to creating new ones, to
keeping track of existing artifacts. 
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebastion or Matt).
'''

import rospy
from basestation_gui_python.msg import Artifact
import copy

class ArtifactHandler:
    '''
    Class that keeps track of artifacts and contains utility functions
    to make new ones, delete artifacts, etc.
    '''
    def __init__(self):
    	self.all_artifacts = [] 
    	self.queued_artifacts = [] # artifacts currently in the queue
    	self.focused_artifact = [] # artifact being displayed in the manipulator plugin
    	self.archived_artifacts = [] #artifacts deleted from queue but we may want to keep around		
    								 #(future gui development needed to actually use this info)


    	#subscriber and publishers
    	rospy.Subscriber('/gui/generate_new_artifact', Artifact, self.generateNewArtifact)
    	rospy.Subscriber('/gui/focus_on_artifact', Artifact, self.skeletonFunction)
    	rospy.Subscriber('/gui/archive_artifact', Artifact, self.archiveArtifact)
    	rospy.Subscriber('/gui/propose_artifact', Artifact, self.skeletonFunction)
    	rospy.Subscriber('/gui/artifact_to_queue', Artifact, self.skeletonFunction)
    	rospy.Subscriber('/gui/update_artifact_info', Artifact, self.updateArtifactInfo) #location, category, or priority

    	self.to_queue_pub = rospy.Publisher('/gui/artifact_to_queue', Artifact, queue_size = 10)



    def generateNewArtifact(self, msg):
    	'''
    	Generate a new artifact
    	'''

    	if (msg.artifact_report_id == -1): #we're manually generating an artifact, make a new 
    									    #artifact id for it
    		
    		#go find the smallest id, and increment it by 1 to generate new id
    		min_negative_id = 0
    		for artifact in self.all_artifacts:
    			if (artifact.artifact_report_id < min_negative_id):
    				min_negative_id = artifact.artifact_report_id

    		artifact_report_id = min_negative_id - 1

    	else:
    		artifact_report_id = msg.artifact_report_id


    	#generate new artifact
    	artifact = GuiArtifact(original_timestamp = msg.original_timestamp, category = msg.category, \
    					    position = [msg.orig_pose.position.x, msg.orig_pose.position.y, msg.orig_pose.position.z],
				            source_robot_id = msg.source_robot_id, artifact_report_id = artifact_report_id, \
				            imgs = msg.imgs, img_stamps = msg.img_stamps)

    	self.all_artifacts.append(artifact)

    	#add the artifact to the queue
    	self.to_queue_pub.publish(msg)

    	self.queued_artifacts.append(artifact)

    def archiveArtifact(self, msg):
    	'''
    	Used when an artifact should eb removed from the queue. 
    	May require a separate plugin to manage such items.
    	'''

    	#go find the artifact
    	artifact_to_archive = None
    	for artifact in self.all_artifacts:
    		if (artifact.unique_id == msg.unique_id):
    			artifact_to_archive = artifact
    			break

    	if (artifact_to_archive != None):
    		self.archived_artifacts.append(artifact)
    		self.queued_artifacts.remove(artifact)

    # def updateArtifactInfo(self, msg):
    # 	'''
    # 	Update the information for an existing artifact
    # 	'''

    def skeletonFunction(self, msg):
    	pass



class GuiArtifact:
    '''
    Class to handle artifacts as an object in the gui
    '''
    def __init__(self, original_timestamp=1, category=-1, position="",
            source_robot_id="", artifact_report_id="", imgs=None, 
            img_stamps=None):
        
        self.category = category
        self.pos = position
        self.orig_pos = copy.deepcopy(position)
        self.source_robot_id = source_robot_id
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


if __name__ == '__main__':
	rospy.init_node('artifact_handler', anonymous=True)
	ArtifactHandler()
	rospy.spin()


    	