#!/usr/bin/python
'''
Functions to handle the artifacts. From detections, to creating new ones, to
keeping track of existing artifacts to submitting artifacts to DARPA
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebastion or Matt).
'''

import rospy
from basestation_gui_python.msg import Artifact, GuiMessage, ArtifactSubmissionReply, WifiDetection, ArtifactDisplayImage, ArtifactUpdate
import copy
from std_msgs.msg import String, UInt8
from darpa_command_post.TeamClient import TeamClient, ArtifactReport
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospkg
import yaml
import cv2


class ArtifactHandler:
	'''
	Class that keeps track of artifacts and contains utility functions
	to make new ones, delete artifacts, etc.
	'''
	def __init__(self):
		self.all_artifacts = {} # dictionary of artifacts, indexed by unique_id
		self.queued_artifacts = {} # dictionary of artifacts currently in the queue
		self.focused_artifact_id = None # artifact being displayed in the manipulator plugin
		self.img_ind_displayed = 0 #the index of the image to display for the artifact focused on
		self.img_displayed = 0 #artifact image index in the set of images to be displayed
		self.archived_artifacts = {} # dictionary ofartifacts deleted from queue but we may want to keep around		
									 #(future gui development needed to actually use this info)
		self.submitted_artifacts = {}

		#read in the artifact categories
		rospack = rospkg.RosPack()
		config_filename = rospack.get_path('basestation_gui_python')+'/config/gui_params.yaml'
		config = yaml.load(open(config_filename, 'r').read())

		darpa_params = config['darpa_params']

		self.artifact_categories = ['Unknown'] #categories from robot are 1-based. so element 0 is unknown
		for category in darpa_params['artifact_categories']:
			self.artifact_categories.append(category)

		self.artifact_priorities = []

		gui_params = config['experiment_params']
		
		for name in gui_params['artifact_priorities']:
			self.artifact_priorities.append(name)

		self.http_client = TeamClient() #client to interact with darpa scoring server
		self.br = CvBridge() #bridge from opencv to ros image messages


		#subscriber and publishers
		rospy.Subscriber('/gui/generate_new_artifact_manual', Artifact, self.generateNewArtifactManually)
		rospy.Subscriber('/gui/focus_on_artifact', String, self.setFocusedArtifact)
		rospy.Subscriber('/gui/archive_artifact', String, self.archiveArtifact)
		rospy.Subscriber('/gui/duplicate_artifact', String, self.duplicateArtifact)
		rospy.Subscriber('/gui/artifact_to_queue', Artifact, self.skeletonFunction)
		rospy.Subscriber('/gui/submit_artifact', String, self.submitArtifact)
		rospy.Subscriber('/gui/change_disp_img', UInt8, self.getArtifactImage) # to handle button presses iterating over artifact images
		rospy.Subscriber('/gui/wifi_detection', WifiDetection, self.handleWifiDetection)
		rospy.Subscriber('/fake_artifact_imgs', WifiDetection, self.handleWifiDetection)#for fake wifi detections
		rospy.Subscriber('/gui/update_artifact_info', ArtifactUpdate,self.updateArtifactInfo) #for updates from the manipulation panels

		self.message_pub = rospy.Publisher('/gui/message_print', GuiMessage, queue_size=10)
		self.to_queue_pub = rospy.Publisher('/gui/artifact_to_queue', Artifact, queue_size = 10)
		self.reply_pub = rospy.Publisher('/gui/submission_reply', ArtifactSubmissionReply, queue_size = 10)
		self.submission_reply_pub = rospy.Publisher('/gui/submission_reply', ArtifactSubmissionReply, queue_size = 10)
		self.img_display_pub = rospy.Publisher('/gui/img_to_display', ArtifactDisplayImage, queue_size = 10)
		self.manipulation_info_pub = rospy.Publisher('/gui/refresh_manipulation_info', Artifact, queue_size = 10)
		self.update_artifact_in_queue_pub = rospy.Publisher('/gui/update_artifact_in_queue', ArtifactUpdate, queue_size=10) #ot change artifact info in the queue
		self.remove_artifact_from_queue_pub = rospy.Publisher('/gui/remove_artifact_from_queue', String, queue_size=10)

		

	def handleWifiDetection(self, msg):
		'''
		An artifact detection or update has come in over wifi. Generate a 
		new artifact and save to proper dictionaries/lists, or update an existing 
		artifact.
		'''

		msg_unique_id = str(msg.artifact_robot_id)+'/'+str(msg.artifact_report_id)+'/'+str(msg.artifact_stamp.secs)

		if (msg.artifact_type == WifiDetection.ARTIFACT_REMOVE): #we're removing an artifact not adding one
			self.archiveArtifact(String(msg_unique_id))

			#remove the artifact from the queue
			remove_msg = String()
			remove_msg.data = msg_unique_id
			self.remove_artifact_from_queue_pub.publish(remove_msg)

		else: 

			#determine if we already have the artifact and this is an update message
			if (msg_unique_id in self.all_artifacts.keys()):
				self.updateWifiDetection(msg)

			else: #this is a completely new artifact detection
				self.generateNewArtifactWifi(msg)




	def guiArtifactToRos(self, artifact):
		'''
		Converts an GuiArtifact to a ros message
		'''
		msg = Artifact()

		msg.category = artifact.category
		msg.curr_pose.position.x = artifact.pose[0]
		msg.curr_pose.position.y = artifact.pose[1]
		msg.curr_pose.position.z = artifact.pose[2]
		msg.orig_pose.position.x = artifact.orig_pose[0]
		msg.orig_pose.position.y = artifact.orig_pose[1]
		msg.orig_pose.position.z = artifact.orig_pose[2]
		msg.source_robot_id = artifact.source_robot_id
		msg.artifact_report_id = artifact.artifact_report_id
		msg.time_from_robot = artifact.time_from_robot
		msg.time_to_darpa = artifact.time_to_darpa
		msg.unread = artifact.unread
		msg.priority = artifact.priority
		msg.darpa_response = artifact.darpa_response
		msg.img_stamps = artifact.img_stamps
		msg.original_timestamp = artifact.original_timestamp
		msg.unique_id = artifact.unique_id

		#convert the mages from numpy to ros
		ros_imgs = []
		for img in artifact.imgs:
			ros_img = self.br.cv2_to_imgmsg(img)
			ros_imgs.append(ros_img)

		msg.imgs = ros_imgs

		return msg

	def rosArtifactToGuiArtifact(self, msg):
		'''
		Converts a Ros artifact to a GuiArtifact
		'''
		if (msg.unique_id in self.all_artifacts.keys()): #we already have this artifact
			artifact = self.all_artifacts[msg.unique_key]

		else: # we need to generate a new artifact
			if (msg.artifact_report_id == -1): #we're manually generating an artifact, make a new 
												#artifact id for it
				
				#go find the smallest id, and increment it by 1 to generate new id
				min_negative_id = 0

				for artifact_key in self.all_artifacts.keys():

					artifact = self.all_artifacts[artifact_key]

					if (artifact.artifact_report_id < min_negative_id):
						min_negative_id = artifact.artifact_report_id

				artifact_report_id = min_negative_id - 1

			else:
				artifact_report_id = msg.artifact_report_id


			#generate new artifact
			artifact = GuiArtifact(original_timestamp = float(msg.original_timestamp), category = msg.category, \
								pose = [msg.orig_pose.position.x, msg.orig_pose.position.y, msg.orig_pose.position.z],
								source_robot_id = msg.source_robot_id, artifact_report_id = artifact_report_id, \
								imgs = msg.imgs, img_stamps = msg.img_stamps, priority = self.artifact_priorities[1])

		return artifact

	def updateArtifactInfo(self, msg):
		'''
		Update an artifact's info

		msg is a custom ArtifactInfo message
		'''

		artifact = self.all_artifacts[msg.unique_id]


		if (msg.update_type == ArtifactUpdate.PROPERTY_CATEGORY):
			artifact.category = msg.category

			#change the value in the artifact queue
			self.update_artifact_in_queue_pub.publish(msg)

		elif (msg.update_type == ArtifactUpdate.PROPERTY_POSE_X):
			artifact.pose[0] = msg.curr_pose.position.x
		
		elif (msg.update_type == ArtifactUpdate.PROPERTY_POSE_Y):
			artifact.pose[1] = msg.curr_pose.position.y

		elif (msg.update_type == ArtifactUpdate.PROPERTY_POSE_Z):
			artifact.pose[2] = msg.curr_pose.position.z

		elif (msg.update_type == ArtifactUpdate.PROPERTY_PRIORITY):
			artifact.priority = msg.priority

			#change the value in the artifact queue
			self.update_artifact_in_queue_pub.publish(msg)

		else:
			update_msg = GuiMessage()
			update_msg.data = 'We received an update message of unknown type. Artifact not updated'
			update_msg.color = update_msg.COLOR_RED
			self.message_pub.publish(update_msg)



	##############################################################################
	# Functions to support generating artifacts
	##############################################################################

	def updateWifiDetection(self, msg):
		'''
		Update artifact from information detected by the robot
		and trasmitted via WiFi 
		'''
		pass

	def generateNewArtifactWifi(self, msg):
		'''
		Generate a new artifact which has been detected from the robot
		and trasmitted via WiFi 
		'''

		#decode the type
		artifact_category = self.artifact_categories[msg.artifact_type]#msg.category is an int using a agreed-upon convention

		#handle the images
		imgs = []
		img_stamps = []

		for img in msg.imgs:
			cv_image = self.br.imgmsg_to_cv2(img)
			cv2.putText(cv_image, "Timestamp: %f" %
				img.header.stamp.to_sec(),
					(5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
					(255, 255, 255), 1)
			imgs.append(cv_image)
			img_stamps.append(img.header.stamp)

		artifact = GuiArtifact(original_timestamp = msg.artifact_stamp.secs, category = artifact_category, \
							pose = [msg.artifact_x, msg.artifact_y, msg.artifact_z],
							source_robot_id = msg.artifact_robot_id, artifact_report_id = msg.artifact_report_id, \
							imgs = imgs, img_stamps = img_stamps, priority = self.artifact_priorities[1])

		self.bookeepAndPublishNewArtifact(artifact)

		

	def bookeepAndPublishNewArtifact(self, artifact):
		'''
		Add the new artifact to the various lists and publish the
		info that we have a new artofact to the various channels

		Input is a GuiArtifact object
		'''

		self.all_artifacts[artifact.unique_id] = artifact
		self.queued_artifacts[artifact.unique_id] = artifact

		#publish this message to be visualized by plugins
		ros_msg = self.guiArtifactToRos(artifact) #necessary step to fill in some defaults (i.e. priority)
											 #to be used by other parts of the gui
								 

		#add the artifact to the queue
		self.to_queue_pub.publish(ros_msg)



	def generateNewArtifactManually(self, msg):
		'''
		Generate a new artifact from a button press for manually-adding one. 
		Input message is custom ROS artifact message
		'''

		#fill in some of the info for the message being published to add to queue
		artifact = self.rosArtifactToGuiArtifact(msg)

		self.bookeepAndPublishNewArtifact(artifact)		

	def duplicateArtifact(self, msg):
		'''
		Take in the unique id of an artifact to duplicate and 
		duplicates it
		'''

		#go find the artifact
		artifact_to_dup = self.all_artifacts[msg.unique_id]

		if (artifact_to_dup != None):

			#find a unique negative id. manually generated artifacts have a negative id
			negative_id_list = []
			for art in self.all_artifacts:
				if (art.artifact_report_id < 0):
					negative_id_list.append(art.artifact_report_id)

			artifact_id = (len(negative_id_list) + 1) * -1

			#make the robot_id negative as well. 
			art_source_id = (artifact_to_dup.source_robot_id + 1) * -1 #'+1' because one of the robot ids is 0, which won't go negative
													 # with '*-1'

			#generate the artifact object
			artifact = GuiArtifact(copy.deepcopy(artifact_to_dup.original_timestamp), copy.deepcopy(artifact_to_dup.category), \
								copy.deepcopy(artifact_to_dup.pose), art_source_id,
								artifact_id, copy.deepcopy(artifact_to_dup.imgs),
								copy.deepcopy(artifact_to_dup.img_stamps), copy.deepcopy(artifact.priority))

			#add the artifact to the list of queued objects and to the all_artifacts list
			self.queued_artifacts[artifact.unique_id] = artifact
			self.all_artifacts[artifact.unique_id] = artifact

			#publish this message to be visualized by plugins
			ros_msg = self.guiArtifactToRos(artifact) #necessary step to fill in some defaults (i.e. priority)
												 #to be used by other parts of the gui

			#add the artifact to the queue
			self.to_queue_pub.publish(ros_msg)

	##############################################################################
	# Functions to support DARPA artifact proposals
	##############################################################################


	def submitArtifact(self, msg):
		'''
		Submit an artifact report to DARPA
		msg is just a string that's the unqiue_id of the artifact to submit
		'''

		artifact = self.all_artifacts[msg.data]

		if (artifact != None):

			artifact_report = ArtifactReport(x = float(artifact.pose[0]), \
											 y = float(artifact.pose[1]), \
											 z = float(artifact.pose[2]), \
											 type = str(artifact.category))

			results = self.http_client.send_artifact_report(artifact_report)

			if (results != []): #we actually get something back
				artifact_report_reply, http_status, http_reason = results

				proposal_return = [artifact_report_reply['run_clock'], artifact_report_reply['type'], \
								  artifact_report_reply['x'], artifact_report_reply['y'], \
								  artifact_report_reply['z'],  artifact_report_reply['report_status'], \
								  artifact_report_reply['score_change'], http_status, http_reason]

				self.publishSubmissionReply(proposal_return)

				#remove the artifact from the book keeping
				self.queued_artifacts.pop(artifact.unique_id) 
				self.submitted_artifacts[artifact.unique_id] = artifact

				#remove the artifact from the queue
				remove_msg = String()
				remove_msg.data = msg.data
				self.remove_artifact_from_queue_pub.publish(remove_msg)


		else: #we could not find the artifact unique_id
			update_msg = GuiMessage()
			update_msg.data = 'Artifact with unique id: '+str(msg.data)+' not found.'
			update_msg.color = update_msg.COLOR_RED
			self.message_pub.publish(update_msg)


	def publishSubmissionReply(self, proposal_return):
		'''
		Publish the information returned by DARPA about our artifact submission
		'''

		[submission_time_raw, artifact_type, x, y, z, report_status, \
							score_change, http_response, http_reason] = proposal_return

		#publish the message to display in thesubmission reply plugin        
		msg = ArtifactSubmissionReply()
		msg.submission_time_raw = float(submission_time_raw)
		msg.artifact_type = str(artifact_type)
		msg.x = x
		msg.y = y
		msg.z = z
		msg.report_status = str(report_status)
		msg.score_change = score_change
		msg.http_response = str(http_response)
		msg.http_reason = str(http_reason)

		self.submission_reply_pub.publish(msg)

	def archiveArtifact(self, msg):
		'''
		Used when an artifact should be removed from the queue. 
		May require a separate plugin to manage such items.
		Message is a string of the unique_id
		'''

		#go find the artifact
		artifact_to_archive = self.all_artifacts[msg.data]

		update_msg = GuiMessage()

		if (artifact_to_archive != None):
			self.archived_artifacts[artifact_to_archive.unique_id] = artifact_to_archive

			self.queued_artifacts.pop(artifact_to_archive.unique_id) #defauilt return value is None if key not found

			update_msg.data = 'Artifact archived in handler:'+str(artifact_to_archive.source_robot_id)+'//'+\
												   str(artifact_to_archive.original_timestamp)+'//'+\
												   str(artifact_to_archive.category)

			update_msg.color = update_msg.COLOR_GREEN
			self.message_pub.publish(update_msg)

		else:
			update_msg.data = 'Artifact not removed from handler'
			update_msg.color = update_msg.COLOR_RED
			self.message_pub.publish(update_msg)

	##############################################################################
	# Functions to support artifact manipulation/visualization
	##############################################################################

	def setFocusedArtifact(self, msg):
		'''
		Set the artifact we're going to be visualizing/manipulating/etc.
		Incoming message is a string of the unique id of the artifact we selected
		'''
		self.focused_artifact_id = msg.data

		self.img_ind_displayed = 0 #reset the image index we're displaying
		self.getArtifactImage(UInt8(2)) #display the first image in the detection

		self.publishManipulationInfomation(msg.data)



	def publishManipulationInfomation(self, unique_id):
		'''
		Send out the artifact manipulation data (original pose and current pose)

		msg is a string that is the unique_id for the artifact
		'''

		msg = self.guiArtifactToRos(self.all_artifacts[unique_id])
		self.manipulation_info_pub.publish(msg)


	def getArtifactImage(self, msg):
		'''
		Get another artifact image to show. 

		msg.data defines whethere we go forward in the set or backward
			0 means go forward
			1 means go backward
			2 means display the first image
		'''

		direction = msg.data #

		#check for errors with request
		update_msg = GuiMessage()

		if (direction not in [0,1,2]):
			update_msg.data = 'Somehow a wrong direction was sent. Image not changed.'
			update_msg.color = update_msg.COLOR_RED
			self.message_pub.publish(update_msg)

		if (self.focused_artifact_id == None):
			update_msg.data = 'No artifact has been selected. Please select one from the queue'
			update_msg.color = update_msg.COLOR_ORANGE
			self.message_pub.publish(update_msg)
			return

		if (len(self.all_artifacts[self.focused_artifact_id].imgs) == 0): #display a black image because this artifact has no images
			self.publishImgToDisplay(-1)

		elif (direction == 0):

			if (self.img_ind_displayed < (len(self.all_artifacts[self.focused_artifact_id].imgs) - 1)):
				#we have some runway to go forward in the sequence
				self.img_ind_displayed += 1

			else:
				#we can't go forward anymore, loop back around
				self.img_ind_displayed = 0

			self.publishImgToDisplay(self.img_ind_displayed)

		elif (direction == 1):

			if (self.img_ind_displayed > 0):
				#we have some runway to go backward in the sequence
				self.img_ind_displayed -= 1
			else:
				#we're already at the beginning on thre sequence, loop back around
				self.img_ind_displayed = len(self.all_artifacts[self.focused_artifact_id].imgs) - 1

			self.publishImgToDisplay(self.img_ind_displayed)

		elif (direction == 2 and (len(self.all_artifacts[self.focused_artifact_id].imgs)>0)): #display the first image
			self.publishImgToDisplay(0)



	def publishImgToDisplay(self, ind):
		'''
		Publish an image to be displayed in the artifact manipulation panel
		'''

		#error checking
		if (self.focused_artifact_id == None):
			update_msg.data = 'No artifact has been selected. Please select one from the queue'
			update_msg.color = update_msg.COLOR_ORANGE
			self.message_pub.publish(update_msg)
			return

		msg = ArtifactDisplayImage()

		if (ind == -1): # this artifact contains no images display the blank black image

			rospack = rospkg.RosPack()
			img_filename = rospack.get_path('basestation_gui_python')+'/src/black_img.png'

			img = cv2.imread(img_filename)

			msg.img = self.br.cv2_to_imgmsg(img)
			msg.image_ind = 0
			msg.num_images = 0

		else:
			#publish the image
			msg.img = self.br.cv2_to_imgmsg(self.all_artifacts[self.focused_artifact_id].imgs[ind])
			msg.image_ind = ind
			msg.num_images = len(self.all_artifacts[self.focused_artifact_id].imgs)
		
		self.img_display_pub.publish(msg)



	def skeletonFunction(self, msg):
		pass



class GuiArtifact:
	'''
	Class to handle artifacts as an object in the gui
	'''
	def __init__(self, original_timestamp=1, category=-1, pose="",
			source_robot_id="", artifact_report_id="", imgs=None, 
			img_stamps=None, priority=None):
		
		
		self.category = category
		self.pose = pose
		self.orig_pose = copy.deepcopy(pose)
		self.source_robot_id = source_robot_id
		self.artifact_report_id = artifact_report_id
		self.time_from_robot = -1 #time the detection has come in from the robot. TODO: change to be something different?
		self.time_to_darpa = -1 #time submitted to darpa
		self.unread = True
		self.priority = priority
		self.darpa_response = ''
		self.imgs = imgs if imgs is not None else []
		self.img_stamps = img_stamps if img_stamps is not None else []
		self.original_timestamp = original_timestamp
		self.unique_id = str(source_robot_id)+'/'+str(artifact_report_id)+'/'+str(original_timestamp)


if __name__ == '__main__':
	rospy.init_node('artifact_handler', anonymous=True)
	ArtifactHandler()
	rospy.spin()


		