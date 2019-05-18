#!/usr/bin/python
'''
Functions to handle the artifacts. From detections, to creating new ones, to
keeping track of existing artifacts to submitting artifacts to DARPA
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebastion or Matt).
'''

import rospy
from basestation_gui_python.msg import Artifact, GuiMessage, ArtifactSubmissionReply
import copy
from std_msgs.msg import String
from darpa_command_post.TeamClient import TeamClient, ArtifactReport

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
		self.submitted_artifacts = []

		#subscriber and publishers
		rospy.Subscriber('/gui/generate_new_artifact', Artifact, self.generateNewArtifact)
		rospy.Subscriber('/gui/focus_on_artifact', Artifact, self.skeletonFunction)
		rospy.Subscriber('/gui/archive_artifact', String, self.archiveArtifact)
		rospy.Subscriber('/gui/duplicate_artifact', String, self.duplicateArtifact)
		rospy.Subscriber('/gui/propose_artifact', Artifact, self.skeletonFunction)
		rospy.Subscriber('/gui/artifact_to_queue', Artifact, self.skeletonFunction)
		rospy.Subscriber('/gui/update_artifact_info', Artifact, self.updateArtifactInfo) #location, category, or priority
		rospy.Subscriber('/gui/submit_artifact', String, self.submitArtifact)

		self.message_pub = rospy.Publisher('/gui/message_print', GuiMessage, queue_size=10)
		self.to_queue_pub = rospy.Publisher('/gui/artifact_to_queue', Artifact, queue_size = 10)
		self.reply_pub = rospy.Publisher('/gui/submission_reply', ArtifactSubmissionReply, queue_size = 10)
		self.submission_reply_pub = rospy.Publisher('/gui/submission_reply', ArtifactSubmissionReply, queue_size = 10)

		self.http_client = TeamClient() #client to interact with darpa scoring server

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
		msg.imgs = artifact.imgs
		msg.img_stamps = artifact.img_stamps
		msg.original_timestamp = artifact.original_timestamp
		msg.unique_id = artifact.unique_id

		return msg

	def rosArtifactToGuiArtifact(self, msg):
		'''
		Converts a Ros artifact to a GuiArtifact
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
							pose = [msg.orig_pose.position.x, msg.orig_pose.position.y, msg.orig_pose.position.z],
							source_robot_id = msg.source_robot_id, artifact_report_id = artifact_report_id, \
							imgs = msg.imgs, img_stamps = msg.img_stamps)

		return artifact

	def generateNewArtifact(self, msg):
		'''
		Generate a new artifact
		'''

		#fill in some of the info for the message being published to add to queue
		artifact = self.rosArtifactToGuiArtifact(msg)

		self.all_artifacts.append(artifact)
		self.queued_artifacts.append(artifact)

		#publish this message to be visualized by plugins
		ros_msg = self.guiArtifactToRos(artifact) #necessary step to fill in some defaults (i.e. priority)
											 #to be used by other parts of the gui

		#add the artifact to the queue
		self.to_queue_pub.publish(ros_msg)

	def findArtifact(self, unique_id):
		'''
		Given a uniqueid, return the artifact being referenced
		'''

		#go find the artifact
		ret_artifact = None
		for artifact in self.all_artifacts:
			if (artifact.unique_id == unique_id):
				ret_artifact = artifact
				break

		if (ret_artifact != None):
			return ret_artifact

		else:
			update_msg = GuiMessage()
			update_msg.data = 'Artifact with unique id: '+str(unique_id)+' not found.'
			update_msg.color = update_msg.COLOR_RED
			self.message_pub.publish(update_msg)

			return None


	def duplicateArtifact(self, msg):
		'''
		Take in the unique id of an artifact to duplicate and 
		duplicates it
		'''

		#go find the artifact
		artifact_to_dup = self.findArtifact(msg.data)

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
								copy.deepcopy(artifact_to_dup.img_stamps))

		   #  GuiArtifact(original_timestamp = msg.original_timestamp, category = msg.category, \
								# pose = [msg.orig_pose.position.x, msg.orig_pose.position.y, msg.orig_pose.position.z],
								# source_robot_id = msg.source_robot_id, artifact_report_id = artifact_report_id, \
								# imgs = msg.imgs, img_stamps = msg.img_stamps)

			#add the artifact to the list of queued objects and to the all_artifacts list
			self.queued_artifacts.append(artifact)
			self.all_artifacts.append(artifact)

			#publish this message to be visualized by plugins
			ros_msg = self.guiArtifactToRos(artifact) #necessary step to fill in some defaults (i.e. priority)
												 #to be used by other parts of the gui

			#add the artifact to the queue
			self.to_queue_pub.publish(ros_msg)


	def submitArtifact(self, msg):
		'''
		Submit an artifact report to DARPA
		msg is just a string that's the unqiue_id of the artifact to submit
		'''

		artifact = self.findArtifact(msg.data)

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
				self.queued_artifacts.remove(artifact)
				self.submitted_artifacts.append(artifact)

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
		'''

		#go find the artifact
		artifact_to_archive = self.findArtifact(msg.data)

		update_msg = GuiMessage()

		if (artifact_to_archive != None):
			self.archived_artifacts.append(artifact_to_archive)
			self.queued_artifacts.remove(artifact_to_archive)

			update_msg.data = 'Artifact archived in handler:'+str(artifact_to_archive.source_robot_id)+'//'+\
												   str(artifact_to_archive.original_timestamp)+'//'+\
												   str(artifact_to_archive.category)

			update_msg.color = update_msg.COLOR_GREEN
			self.message_pub.publish(update_msg)

		else:
			update_msg.data = 'Artifact not removed from handler'
			update_msg.color = update_msg.COLOR_RED
			self.message_pub.publish(update_msg)

	def updateArtifactInfo(self, msg):
		'''
		Update the information for an existing artifact
		'''
		pass

	def skeletonFunction(self, msg):
		pass



class GuiArtifact:
	'''
	Class to handle artifacts as an object in the gui
	'''
	def __init__(self, original_timestamp=1, category=-1, pose="",
			source_robot_id="", artifact_report_id="", imgs=None, 
			img_stamps=None):
		
		self.category = category
		self.pose = pose
		self.orig_pose = copy.deepcopy(pose)
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


		