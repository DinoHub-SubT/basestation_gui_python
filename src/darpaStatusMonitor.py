#!/usr/bin/env python

'''
A node for getting status information (time, score, remaining reports) from DARPA
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University (CMU) / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebastion or Matt).
'''

import rospy
from std_msgs.msg import String, Float32MultiArray
import yaml
import sys
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
import pdb
from basestation_gui_python.msg import RadioMsg, NineHundredRadioMsg, DarpaStatus
from darpa_command_post.TeamClient import TeamClient, ArtifactReport
import threading
import time
from visualization_msgs.msg import InteractiveMarkerFeedback, MarkerArray, Marker

class DarpaBridge:
	def __init__(self, config_filename):

		# parse the config file
		config = yaml.load(open(config_filename, 'r').read())
		
		#read info on the experiment parameters (# of robots, etc.)
		darpa_params = config['darpa_params']
		

		self.auth_bearer_token = darpa_params['auth_bearer_token'][0]
		self.request_info_uri = darpa_params['scoring_uris'][0] #uri for requesting information (time,score,etc) from darpa
		# self.post_artifact_uri = darpa_params['scoring_uris'][1] #uri for posting artifact proposals to DARPA

		#have an initial darpa status update
		self.darpa_status_update = {}
		self.darpa_status_update['run_clock'] = None
		self.darpa_status_update['score'] = None
		self.darpa_status_update['remaining_reports'] = None

		#if we're also simulating the darpa command post
		self.connect_to_command_post = rospy.get_param("/connect_to_command_post")            


		#start a schedule which runs "get status" every few seconds
		if(self.connect_to_command_post):
			self.http_client = TeamClient()
			self.get_status_thread = threading.Timer(1.0, self.getStatus)
			self.get_status_thread.start()

			self.status_pub = rospy.Publisher('/gui/darpa_status', DarpaStatus, queue_size=10)


	def getStatus(self):
		'''
		Function that calls the http code to get the status and published to appropriate ros topic
		'''
		self.darpa_status_update = self.http_client.get_status_from_command_post()

		try:
			r = float(self.darpa_status_update['run_clock'])
		except:
			self.darpa_status_update['run_clock'] = 999

		#publish this data as something
		time_remaining = self.displaySeconds(float(self.darpa_status_update['run_clock']))
		
		msg = DarpaStatus()

		msg.time_elapsed = float(self.darpa_status_update['run_clock'])
		msg.score = self.darpa_status_update['score']
		msg.remaining_reports = self.darpa_status_update['remaining_reports']


		self.status_pub.publish(msg)

		#restart the thread to call this function again
		self.get_status_thread = threading.Timer(1.0, self.getStatus)
		self.get_status_thread.start()

	def shutdownHttpServer(self):
		'''
		Closes the http connection
		'''

		#stop the thread that keeps reuesting the status
		self.get_status_thread.cancel()


		self.http_client.exit()

	def displaySeconds(self, seconds):
		'''
		Function to convert seconds float into a min:sec string
		'''
		#convert strings to floats
		seconds = float(seconds)

		seconds_int = int(seconds-(int(seconds)/60)*60)
		if seconds_int < 10:
			seconds_str = '0'+str(seconds_int)
		else:
			seconds_str = str(seconds_int)

		return str((int(seconds)/60))+':'+seconds_str

if __name__ == '__main__':

	rospy.init_node('darpa_status_node', anonymous=True)
	
	config_filename = sys.argv[1]

	ros_gui_bridge = DarpaBridge(config_filename)    

	rospy.spin()