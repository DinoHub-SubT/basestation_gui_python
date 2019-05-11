#!/usr/bin/python

'''
Plugin for buttons pertaining to robot commands
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebastion or Matt).
'''

import rospy
import rospkg
from std_msgs.msg import String
import threading

from qt_gui.plugin import Plugin
import python_qt_binding.QtWidgets as qt
import python_qt_binding.QtCore as core
import python_qt_binding.QtGui as gui

from python_qt_binding import QT_BINDING, QT_BINDING_VERSION

try:
	from pkg_resources import parse_version
except:
	import re

	def parse_version(s):
		return [int(x) for x in re.sub(r'(\.0+)*$', '', s).split('.')]

if QT_BINDING == 'pyside':
	qt_binding_version = QT_BINDING_VERSION.replace('~', '-')
	if parse_version(qt_binding_version) <= parse_version('1.1.2'):
		raise ImportError('A PySide version newer than 1.1.0 is required.')

from python_qt_binding.QtCore import Slot, Qt, qVersion, qWarning, Signal
from python_qt_binding.QtGui import QColor, QPixmap
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy

from GuiBridges import RosGuiBridge, DarpaGuiBridge
from functools import partial
import pdb
import yaml
import numpy as np
from GuiEngine import GuiEngine, Artifact
from PyQt5.QtCore import pyqtSignal

from basestation_gui_python.msg import GuiMessage, DarpaStatus, GuiRobCommand, RadioMsg

class RobotCommandPlugin(Plugin):

	rob_command_trigger = pyqtSignal(object) #to keep the drawing on the proper thread

	def __init__(self, context):
		super(RobotCommandPlugin, self).__init__(context)
		self.setObjectName('RobotCommandPlugin')

		#get the number of robots
		rospack = rospkg.RosPack()
		config_filename = rospack.get_path('basestation_gui_python')+'/config/gui_params.yaml'
		config = yaml.load(open(config_filename, 'r').read())

		self.robot_names, self.ground_commands, self.aerial_commands = [], [], []

		exp_params = config['experiment_params']
		
		for name in exp_params['robot_names']:
			self.robot_names.append(name)

		for name in exp_params['ground_commands']:
			self.ground_commands.append(name)

		for name in exp_params['aerial_commands']:
			self.aerial_commands.append(name)

		#denote e-stop commands
		self.ground_estop_commands = self.ground_commands[:4]
		self.aerial_estop_commands = self.aerial_commands[:4]

		self.initPanel(context) #layout plugin

		#setup subscribers/publishers
		self.rob_command_sub = rospy.Subscriber('/gui_rob_command_update', GuiRobCommand, self.robCommandUpdate)
		self.rob_command_pub = rospy.Publisher('/gui_rob_command_press', GuiRobCommand, queue_size = 10)
		self.radio_pub = rospy.Publisher('/from_gui', RadioMsg, queue_size=50) #queue_size arbitraily chosen

		self.rob_command_trigger.connect(self.robCommandUpdateMonitor)
		

	def initPanel(self, context):
		'''
		Initialize the panel for displaying widgets
		'''

		#define the overall plugin
		self.widget = QWidget()
		self.global_widget = qt.QGridLayout()     
		
		self.widget.setLayout(self.global_widget)
		context.add_widget(self.widget)

		self.control_widget = QWidget()
		self.control_layout = qt.QGridLayout()

		#define the overall widget
		control_label = qt.QLabel()
		control_label.setText('CONTROL PANEL')
		control_label.setAlignment(Qt.AlignCenter)
		self.control_layout.addWidget(control_label)

		#define the number of commands in a single column
		num_in_col = 6
		self.control_buttons = []

		#establish the sub-panel for each robot
		for robot_num, robot_name in enumerate(self.robot_names):

			#define the layout and group for a single robot
			robot_layout = qt.QGridLayout()
			robot_groupbox = qt.QGroupBox("Controls "+robot_name)
			robot_button_list = []

			#add the robot commands
			row, col = 0, 0 #the row and column to put the buttons

			#command dependent on type of robot
			if (robot_name.find('erial')!=-1):
				commands = self.aerial_commands
			else:
				commands = self.ground_commands

			for j, command in enumerate(commands):
				
				button = qt.QPushButton(command)
				robot_button_list.append(button)

				self.styleButton(button) #color, click properties, etc.

				#upon press, do something in ROS
				button.clicked.connect(partial(self.processRobotCommandPress, command, robot_name, button))

				robot_layout.addWidget(button, row, col)

				#if we have filled this column, move to the next
				if(row==(num_in_col-1)): 
					row = 0
					col+=1
				else:
					row+=1

			#add custom buttons

			#add a combobox to set the max run time for the aerial vehicle
			if (robot_name.find('erial') != -1):
				max_time_list = np.arange(0., 10.5, 0.5).tolist()
				max_time_box = qt.QComboBox()

				for speed in max_time_list:
					max_time_box.addItem(str(speed))
				
				max_time_box.currentTextChanged.connect(partial(self.adjustMaxTime, robot_name, max_time_box)) 

				robot_layout.addWidget(max_time_box, row, col)

			#add buttons in robot panel

			self.control_buttons.append(robot_button_list)

			robot_groupbox.setLayout(robot_layout)
			self.control_layout.addWidget(robot_groupbox)

		#add to the overall gui
		self.control_widget.setLayout(self.control_layout)
		self.global_widget.addWidget(self.control_widget)

	def styleButton(self, button):
		'''
		Add properties to the button like color, etc.
		'''
		command = button.text()

		if (command in ['Resume', 'Resume/Takeoff']):
			button.setCheckable(True) # a button pressed will stay pressed, until unclicked
			button.setStyleSheet("QPushButton:checked { background-color: green }") #a button stays green when its in a clicked state

		elif (command in ['Confirm', 'Cancel']):
			button.setStyleSheet("background-color:rgb(126, 126, 126)")
			button.setEnabled(False)

		elif (command in ['Return home', 'Drop comms']):
			button.setStyleSheet("QPushButton:pressed { background-color: red }") #a button stays red when its in a clicked state
			
		else:
			button.setCheckable(True) # a button pressed will stay pressed, until unclicked
			button.setStyleSheet("QPushButton:checked { background-color: red }") #a button stays red when its in a clicked state

	def processRobotCommandPress(self, command, robot_name, button):
		'''
		Handle button colors, etc. upon pressing
		Generate and publish a ROS command from a button press
		'''
		self.processButtonStyle(command, robot_name, button)
		self.sendRosCommandMsg(command, robot_name, button)

	def processButtonStyle(self, command, robot_name, button):
		'''
		Handle the button color, behvior, etc. upon pressing
		'''
		#if its an estop button, de-activate all other estop buttons
		if ((robot_name.find('ound') != -1) and (button.text() in self.ground_estop_commands)) or\
			 ((robot_name.find('erial') != -1) and (button.text() in self.aerial_estop_commands)):

			#find the set of control buttons for this robot
			robot_ind = -1
			for i, name in enumerate(self.robot_names):
				if (robot_name == name):
					robot_ind = i

			if (robot_ind!=-1):
				control_buttons = self.control_buttons[robot_ind]

				for cmd_button in control_buttons:
					if (cmd_button != button):                        
						cmd_button.setChecked(False)

		# if (command == 'Return to comms']):
		# 	button.setText('')

		# 	 or 'Land in comms'
					

	def sendRosCommandMsg(self, command, robot_name, button):
		'''
		Alert the system via ROS msg that a command button has been pressed
		'''
		pass


	
	def robCommandUpdate(self, msg):
		self.skeleton_trigger.emit(msg)


	def robCommandUpdateMonitor(self, msg):
		'''
		Draw something on the gui in this function
		'''
		#check that threading is working properly
		if (not isinstance(threading.current_thread(), threading._MainThread)):
			print "Drawing on the message panel not guarented to be on the proper thread"	

	def adjustMaxTime(self, robot_name, max_time_box):
		'''
		Send a maxtime for the aerial vehicle to fly
		'''
		radio_msg = RadioMsg()
		radio_msg.message_type = RadioMsg.MESSAGE_TYPE_MAX_FLIGHT_TIME
		radio_msg.recipient_robot_id = self.robot_names.index(robot_name)
		radio_msg.data = str(float(max_time_box.currentText())*60)

		self.radio_pub.publish(radio_msg)		

			
	def shutdown_plugin(self):
		# TODO unregister all publishers here
		self.rob_command_sub.unregister()
		
