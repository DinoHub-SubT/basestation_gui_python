#!/usr/bin/python

'''
Plugin containing buttons that are relevant to the entire basestation gui.
Preliminarily, this includes a big red estop button for the robots
and a load-from-csv button
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebastion or Matt).
'''

import rospy
import rospkg
from std_msgs.msg import String, Bool
import threading

from qt_gui.plugin import Plugin
import python_qt_binding.QtWidgets as qt
import python_qt_binding.QtCore as core
import python_qt_binding.QtGui as gui

from python_qt_binding import QT_BINDING, QT_BINDING_VERSION


from python_qt_binding.QtCore import Slot, Qt, qVersion, qWarning, Signal
from python_qt_binding.QtGui import QColor, QPixmap
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy

import pdb
import yaml
from PyQt5.QtCore import pyqtSignal

from basestation_gui_python.msg import GuiMessage, DarpaStatus, RadioMsg, Artifact

class GlobalButtonsPlugin(Plugin):

	def __init__(self, context):
		super(GlobalButtonsPlugin, self).__init__(context)
		self.setObjectName('GlobalButtonsPlugin')		

		#get the number of robots
		rospack = rospkg.RosPack()
		config_filename = rospack.get_path('basestation_gui_python')+'/config/gui_params.yaml'
		config = yaml.load(open(config_filename, 'r').read())

		#get the robot names
		self.robot_names = []

		exp_params = config['experiment_params']
		
		for name in exp_params['robot_names']:
			self.robot_names.append(name)

		self.initPanel(context) #layout plugin

		#setup subscribers/publishers
		self.radio_pub = rospy.Publisher('/from_gui', RadioMsg, queue_size = 10) #for sending estop commands to the robot
		self.big_red_pub = rospy.Publisher('/gui/global_estop', Bool, queue_size=10) #for telling the other plugins we jsut sen estop commands


	def initPanel(self, context):
		'''
		Initialize the panel for displaying widgets
		'''		

		#define the overall widget
		self.global_buttons_widget = QWidget()
		self.global_buttons_layout = qt.QVBoxLayout()

		context.add_widget(self.global_buttons_widget)

		button = qt.QPushButton("SOFT ESTOP ALL\n  ROBOTS")
		button.setStyleSheet("background-color: red")
		button.clicked.connect(self.processBigRed)
		self.global_buttons_layout.addWidget(button)     

		#add to the overall gui
		self.global_buttons_widget.setLayout(self.global_buttons_layout)
	
	def processBigRed(self):
		'''
		The big red e-stop button was pressed. 
		Publish a soft e-stop to all of the robots.
		'''	

		radio_msg = RadioMsg()
		radio_msg.message_type = RadioMsg.MESSAGE_TYPE_ESTOP
		radio_msg.data = RadioMsg.ESTOP_SOFT

		for i, robot_name in enumerate(self.robot_names):

			#send out the soft estop commands
			radio_msg.recipient_robot_id = self.robot_names.index(robot_name)
			self.radio_pub.publish(radio_msg)

			#send out a message to the robot command panel indicating we just did
			#this and the proper buttons should be pressed/un-pressed
			self.big_red_pub.publish(Bool(True))

			
	def shutdown_plugin(self):
		# TODO unregister all publishers here
		pass
		
