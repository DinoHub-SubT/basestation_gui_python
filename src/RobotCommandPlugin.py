#!/usr/bin/python

'''
Plugin for buttons pertaining to robot commands
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebastion or Matt).
'''

import rospy
import rospkg
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
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
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy, QTabWidget

from GuiBridges import RosGuiBridge, DarpaGuiBridge
from functools import partial
import pdb
import yaml
import numpy as np
from GuiEngine import GuiEngine, Artifact
from PyQt5.QtCore import pyqtSignal

from basestation_gui_python.msg import GuiMessage, DarpaStatus, GuiRobCommand, RadioMsg, NineHundredRadioMsg
from visualization_msgs.msg import InteractiveMarkerFeedback, MarkerArray, Marker

class RobotCommandPlugin(Plugin):

	rob_command_trigger = pyqtSignal(object) #to keep the drawing on the proper thread

	def __init__(self, context):
		super(RobotCommandPlugin, self).__init__(context)
		self.setObjectName('RobotCommandPlugin')

		#get the number of robots
		rospack = rospkg.RosPack()
		config_filename = rospack.get_path('basestation_gui_python')+'/config/gui_params.yaml'
		config = yaml.load(open(config_filename, 'r').read())

		self.robot_names, self.ground_commands, self.aerial_commands, self.robot_pos_topics = [], [], [], []

		exp_params = config['experiment_params']
		
		for name in exp_params['robot_names']:
			self.robot_names.append(name)

		for name in exp_params['ground_commands']:
			self.ground_commands.append(name)

		for name in exp_params['aerial_commands']:
			self.aerial_commands.append(name)

		for name in exp_params['robot_pos_topics']:
			self.robot_pos_topics.append(name)

		#robot names should be unique
		if(len(np.unique(self.robot_names))!=len(self.robot_names)):
			raise ValueError('Not all of the robot names are unique!')

		#denote e-stop commands
		self.ground_estop_commands = self.ground_commands[:4]
		self.aerial_estop_commands = self.aerial_commands[:4]

		self.initPanel(context) #layout plugin

		self.robot_positions = [None]*len(self.robot_names)
		self.rob_command_trigger.connect(self.robCommandUpdateMonitor)

		#setup subscribers/publishers
		self.rob_command_sub = rospy.Subscriber('/gui/rob_command_update', GuiRobCommand, self.robCommandUpdate)
		self.waypoint_sub = rospy.Subscriber('/define_waypoint/feedback', InteractiveMarkerFeedback, self.recordWaypoint)
		rospy.Subscriber('/gui/global_estop', Bool, self.processGlobalEstopCommand)

		self.waypoint = None   
		for i, topic in enumerate(self.robot_pos_topics):
			rospy.Subscriber(topic, Odometry, self.saveRobotPos, (i))


		

		self.rob_command_pub = rospy.Publisher('/gui/rob_command_press', GuiRobCommand, queue_size = 10)
		self.radio_pub = rospy.Publisher('/from_gui', RadioMsg, queue_size=50) #queue_size arbitraily chosen
		self.gui_message_pub = rospy.Publisher('/gui/message_print', GuiMessage, queue_size=10)
		self.radio_900_pub = rospy.Publisher('/ros_to_teensy', NineHundredRadioMsg, queue_size=50) #queue_size arbitraily chosen
		self.marker_orig_pos_pub = rospy.Publisher('/refinement_marker_orig_pos', MarkerArray, queue_size=50)#for displaying the original position 
		self.highlight_robot_pub = rospy.Publisher('/highlight_robot_pub', Marker, queue_size=50)#for displaying robot position           
		self.bluetooth_marker_pub_ugv = rospy.Publisher('/ugv/bluetooth_marker', Marker, queue_size=50) #for displaying bluetooth detections       
		self.bluetooth_marker_pub_uav = rospy.Publisher('/uav/bluetooth_marker', Marker, queue_size=50) #for displaying bluetooth detections
		self.define_waypoint_marker_pos_pub = rospy.Publisher('/define_waypoint_marker_pos', Point, queue_size=50)#publisher for moving the refinment marker
		self.define_waypoint_marker_off_pub = rospy.Publisher('/define_waypoint_marker_off', Point, queue_size=50)#publisher for turning the marker off

		

		

		
		

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

		#define the tab widget which the other widgets will go in
		self.tabs = QTabWidget()

		#define the number of commands in a single column
		num_in_col = 7
		self.control_buttons = []

		#establish the sub-panel for each robot
		for robot_num, robot_name in enumerate(self.robot_names):

			#define the layout and group for a single robot
			robot_layout = qt.QGridLayout()
			robot_tab = QWidget()
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

			#add a combobox to set the max run time for the vehicle
			max_time_list = np.arange(0., 10.5, 0.5).tolist()
			max_time_box = qt.QComboBox()

			for speed in max_time_list:
				max_time_box.addItem(str(speed))
			
			max_time_box.currentTextChanged.connect(partial(self.adjustMaxTime, robot_name, max_time_box)) 

			robot_layout.addWidget(max_time_box, row, col)

			#add buttons in robot panel
			self.control_buttons.append(robot_button_list)

			robot_tab.setLayout(robot_layout)
			self.tabs.addTab(robot_tab, robot_name)

		#add to the overall gui
		self.control_layout.addWidget(self.tabs)
		self.control_widget.setLayout(self.control_layout)
		self.global_widget.addWidget(self.control_widget)

	def saveRobotPos(self, msg, robot_ind):
		'''
		Save the robot position for define waypoint functionality
		'''
		self.robot_positions[robot_ind] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]

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

		elif (command in ['Return home', 'Drop comms', 'Re-send artifacts']):
			button.setStyleSheet("QPushButton:pressed { background-color: green }") #a button stays red when its in a clicked state
			
		else:
			button.setCheckable(True) # a button pressed will stay pressed, until unclicked
			button.setStyleSheet("QPushButton:checked { background-color: red }") #a button stays red when its in a clicked state

	def processGlobalEstopCommand(self, msg):
		'''
		The big red button (a global estop button) has just been pressed.
		The commands have already been sent via radio. We now just need to
		press/un-press the proper estop buttons

		msg is just a boolean indicating whether we should do this or not
		its always True
		'''

		if (msg.data == True):
			for robot_buttons in self.control_buttons:
				for button in robot_buttons:

					#check to see if a soft-estop button. if so, press it
					if (button.text() == self.ground_estop_commands[2]) or \
					   (button.text() == self.aerial_estop_commands[2]):

					   button.setChecked(True)


					#if its an estop button that is not soft estop, un-press it
					elif ((button.text() in self.ground_estop_commands) or\
					    (button.text() in self.aerial_estop_commands)): 

						button.setChecked(False)




	def processRobotCommandPress(self, command, robot_name, button):
		'''
		Handle button colors, etc. upon pressing
		Generate and publish a ROS command from a button press
		'''

		#if its a button that needs confirmation, or its a confirmation button,
		#follow a different procedure
		#if its a button that needs confirmation, get that confirmation
		if button.isChecked() and \
			 ((command =='Hard e-stop') or (robot_name.find('erial') != -1 and command == 'Land')):

			self.processButtonNeedingConfirmation(command, robot_name, button)
			self.pending_info =[command, robot_name, button]

		#we have just confirmed or cancelled a previous button press
		elif (command in ['Cancel', 'Confirm']):
			self.processConfirmCancelSequence(command, robot_name, button, self.pending_info)
			

		#if its a normal button
		else:
			self.processButtonStyle(command, robot_name, button)
			self.publishRobotCommand(command, robot_name, button)


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

		#change the text for the return button
		if ((robot_name.find('ound') != -1) and button.text() == 'Return to comms'):
			button.setText('Explore forever')

		elif ((robot_name.find('ound') != -1) and button.text() == 'Explore forever'):
			button.setText('Return to comms')

		if ((robot_name.find('erial') != -1) and button.text() == 'Land in comms'):
			button.setText('Explore forever')

		elif ((robot_name.find('erial') != -1) and button.text() == 'Explore forever'):
			button.setText('Land in comms')

		#de-activate any confirmation buttons that are left over from previous button presses
		cmd_buttons = self.control_buttons[self.robot_names.index(robot_name)]

		for cmd_button in cmd_buttons:
			#de-activate the confirm/cancel buttons for that robot
			if (cmd_button.text() in ['Confirm', 'Cancel']): 
				cmd_button.setEnabled(False)
				cmd_button.setStyleSheet("background-color:rgb(126, 126, 126)")

						

	def publishRobotCommand(self, command, robot_name, button):
		'''
		A command button has been pressed. Publish a command from the gui to the robot
		'''

		if (command in [ "Return home", "Highlight robot", "Drop comms", "Re-send artifacts"]): #buttons are not checkable
			if(command == "Return home"):
				self.publishReturnHome(robot_name)
			elif(command == "Highlight robot"):
				self.highlightRobot(robot_name)
			elif(command == "Drop comms"):
				self.dropComms(robot_name)
			elif(command=="Re-send artifacts"):
				self.resendArtifacts(robot_name)
			else:
				msg = GuiMessage()
				msg.data = 'WARNING: Button pressed does not have a function call associated with it!'
				msg.color = msg.COLOR_ORANGE
				self.gui_message_pub.publish(msg)

		elif(not button.isChecked()): #it has just be un-clicked

			if(command == "Define waypoint"):
				#find the robot name index and unsubscribe it
				try:
					self.publishWaypointGoal(robot_name)
				except ValueError:
					msg = GuiMessage()
					msg.data = "Something went wrong registering robot names and the subscriber listening to waypoint definitions may not have been disabled!!"
					msg.color = msg.COLOR_RED
					self.gui_message_pub.publish(msg)

			elif(command == "Show bluetooth"):
				self.handleBluetooth(robot_name, button)

			elif(command in ['Return to comms', 'Land in comms']):
				self.pubLandInComms(robot_name, button)

		else: #the button has just been pressed
			if ((robot_name.find('ound') != -1) and (button.text() in self.ground_estop_commands)) or\
			   ((robot_name.find('erial') != -1) and (button.text() in self.aerial_estop_commands)):

				self.publishEstop(command, robot_name)

			elif(command == "Define waypoint"):
				self.defineWaypoint(robot_name)

			elif(command == "Show bluetooth"):
				self.handleBluetooth(robot_name, button)

			elif(command in ['Return to comms', 'Land in comms']):
				self.pubLandInComms(robot_name, button)

			else:
				msg = GuiMessage()
				msg.data = 'WARNING: Button pressed does not have a function call associated with it!'
				msg.color = msg.COLOR_ORANGE
				self.gui_message_pub.publish(msg)

	def resendArtifacts(self, robot_name):
		'''
		Re-send all fo the artifact detections from the robot.
		Useful when the robot goes out of comms range and comes
		back in, we can have the detections while out of range sent
		back
		'''

		radio_msg = RadioMsg()
		radio_msg.recipient_robot_id = self.robot_names.index(robot_name) 
		radio_msg.message_type = RadioMsg.MESSAGE_TYPE_RESEND_ALL_ARTIFACTS

		self.radio_pub.publish(radio_msg)

	
	def processButtonNeedingConfirmation(self, command, robot_name, button):
		'''
		Enable the confirm/cancel buttons for a pending button press
		'''

		#activate the confirm/cancel buttons for that robot
		cmd_buttons = self.control_buttons[self.robot_names.index(robot_name)]

		for cmd_button in cmd_buttons:
			if (cmd_button.text() =='Confirm'):
				cmd_button.setEnabled(True)
				cmd_button.setStyleSheet("background-color:rgb(0, 220, 0)")

			elif(cmd_button.text() =='Cancel'):
				cmd_button.setEnabled(True)
				cmd_button.setStyleSheet("background-color:rgb(220, 0, 0)")

	def processConfirmCancelSequence(self, command, robot_name, button, pending_info):
		'''
		Process a confirm or cancel on a pending button press
		'''

		[pending_command, pending_robot_name, pending_button] = pending_info

		if (command=='Confirm'):
			self.processButtonStyle(pending_command, pending_robot_name, pending_button)
			self.publishRobotCommand(pending_command, pending_robot_name, pending_button)

		elif (command=='Cancel'):
			#de-activate any confirmation buttons that are left over from previous button presses
			cmd_buttons = self.control_buttons[self.robot_names.index(robot_name)]

			for cmd_button in cmd_buttons:
				#de-activate the confirm/cancel buttons for that robot
				if (cmd_button.text() in ['Confirm', 'Cancel']): 
					cmd_button.setEnabled(False)
					cmd_button.setStyleSheet("background-color:rgb(126, 126, 126)")

			#de-activate the pending button press
			pending_button.setChecked(False)

		else:
			msg = GuiMessage()
			msg.data = 'Something went wrong with confirm/cancel sequence'
			msg.color = msg.COLOR_RED
			self.gui_message_pub.publish(msg)


		self.pending_info = None





	def pubLandInComms(self, robot_name, button):
		'''
		Depending on if the button is pressed or not, send a 
		message to land in comms range or back at home
		'''

		radio_msg = RadioMsg()
		radio_msg.recipient_robot_id = self.robot_names.index(robot_name)
		radio_msg.message_type = RadioMsg.MESSAGE_TYPE_LANDING_BEHAVIOR

		if (button.isChecked()):             
			radio_msg.data = RadioMsg.LAND_IN_COMMS

		else:            
			radio_msg.data = RadioMsg.LAND_AT_HOME

		self.radio_pub.publish(radio_msg)

	def handleBluetooth(self, robot_name, button):
		'''
		Make the bluetooth marker visualizable or hidden.
		TODO: Probably needs to be updated. Will do if 

		'''
		pass
		

	def dropComms(self, robot_name):
		'''
		Send out a message to drop a commas node
		'''
		radio_msg = RadioMsg()
		radio_msg.message_type = RadioMsg.MESSAGE_TYPE_DROP_COMMS
		radio_msg.recipient_robot_id = self.robot_names.index(robot_name)
		self.radio_pub.publish(radio_msg)

	def highlightRobot(self, robot_name):
		'''
		Publish an arrow pointng to the robot position
		TODO: Maybe publish a huge arrow marker, if someone requests this functionality
		'''
		pass


	def publishEstop(self, command, robot_name):
		'''
		Publish an estop message after a button has been pressed
		'''

		radio_msg = RadioMsg()
		send_900 = bool(False)
		radio_900_msg = NineHundredRadioMsg()
		radio_msg.message_type = RadioMsg.MESSAGE_TYPE_ESTOP
		radio_msg.recipient_robot_id = self.robot_names.index(robot_name)
		radio_900_msg.recipient_robot_id = self.robot_names.index(robot_name)

		if (robot_name.find('erial')!=-1):
			estop_commands = self.aerial_estop_commands

		else:
			estop_commands = self.ground_estop_commands

		if(command == estop_commands[1]):
			radio_msg.data = RadioMsg.ESTOP_RESUME
			radio_900_msg.message_type = 0
			send_900 = True

		elif(command == estop_commands[0]):
			radio_msg.data = RadioMsg.ESTOP_PAUSE

		elif(command == estop_commands[2]):
			radio_msg.data = RadioMsg.ESTOP_SOFT

		elif(command == estop_commands[3]):
			radio_msg.data = RadioMsg.ESTOP_HARD
			radio_900_msg.message_type = 1
			send_900 = True

			#make the estop persistent for the drone
			if (robot_name.find('erial') != -1):
				self.drone_hard_estop_thread = threading.Timer(2.0, partial(self.persistentDroneHardEstop, robot_name))
				self.drone_hard_estop_thread.start()

		else:
			msg = GuiMessage()
			msg.data = 'WARNING: The pressed button does not correspond to an estop command the Bridge knows about'
			msg.color = msg.COLOR_ORANGE
			self.gui_message_pub.publish(msg)

		if send_900:
			self.radio_900_pub.publish(radio_900_msg)

		self.radio_pub.publish(radio_msg)

	def persistentDroneHardEstop(self, robot_name):
		'''
		Publish a hard estop every __ seconds for the drone
		'''

		#make sure the button has not been unchecked since the last thread call
		for cmd_button in self.control_buttons[self.robot_names.index(robot_name)]:
			if (cmd_button.text()=='Hard e-stop') and (cmd_button.isChecked()): 

				radio_900_msg = NineHundredRadioMsg()
				radio_900_msg.message_type = 1
				radio_900_msg.recipient_robot_id = self.robot_names.index(robot_name)
				self.radio_900_pub.publish(radio_900_msg)

				radio_msg = RadioMsg()
				radio_msg.message_type = RadioMsg.MESSAGE_TYPE_ESTOP
				radio_msg.recipient_robot_id = self.robot_names.index(robot_name)
				radio_msg.data = RadioMsg.ESTOP_HARD
				self.radio_pub.publish(radio_msg)

				self.drone_hard_estop_thread = threading.Timer(2.0, partial(self.persistentDroneHardEstop, self.robot_names[1])) 
				self.drone_hard_estop_thread.start()

	def publishReturnHome(self, robot_name):
		'''
		Send out a message for the robot to return home
		'''

		radio_msg = RadioMsg()
		radio_msg.message_type = RadioMsg.MESSAGE_TYPE_RETURN_HOME
		radio_msg.recipient_robot_id = self.robot_names.index(robot_name)
		self.radio_pub.publish(radio_msg)

	

	def publishWaypointGoal(self, robot_name):

		if (self.waypoint != None):
			radio_msg = RadioMsg()
			radio_msg.message_type = RadioMsg.MESSAGE_TYPE_DEFINE_WAYPOINT
			radio_msg.recipient_robot_id = self.robot_names.index(robot_name)
			radio_msg.data = str(self.waypoint[0]) +","+str(self.waypoint[1]) +","+str(self.waypoint[2])

			self.radio_pub.publish(radio_msg)

		else:
			msg = GuiMessage()
			msg.data = 'Waypoint never set. Message never published. Please move the interactive marker in RViz.'
			msg.color = msg.COLOR_GRAY
			self.gui_message_pub.publish(msg)


		#publish a bogus message to put something on this topic to turn off the refinment marker
		pose = Point(-1, -1, -1)
		self.define_waypoint_marker_off_pub.publish(pose)




	def defineWaypoint(self, robot_name):
		'''
		Listen for a waypoint to be pressed in Rviz
		'''

		#subscriber for listening to waypoint goals
		try:
			# self.waypoint_listeners[self.robot_names.index(robot_name)] = rospy.Subscriber(self.waypoint_topic, PoseStamped, self.publishWaypointGoal, robot_name)

			#publish the interactive marker
				
			if (self.robot_positions[self.robot_names.index(robot_name)] == None):
				msg = GuiMessage()
				msg.data = 'Nothing appears to have been published to the robot pose topic ' +\
										 self.robot_pos_topics[self.robot_names.index(robot_name)]
				
				msg.color = msg.COLOR_GRAY
				self.gui_message_pub.publish(msg)

				#deselect the waypoint button
				for cmd_button in self.control_buttons[self.robot_names.index(robot_name)]:
					if (cmd_button.text()=='Define waypoint') and (cmd_button.isChecked()): 
						cmd_button.setChecked(False)
			
			else:
				#reset the initial waypoint
				self.waypoint = None

				pose = Point(self.robot_positions[self.robot_names.index(robot_name)][0], \
							 self.robot_positions[self.robot_names.index(robot_name)][1], \
							 self.robot_positions[self.robot_names.index(robot_name)][2]) #put robot pose in here

				self.define_waypoint_marker_pos_pub.publish(pose)

			

		except ValueError:
			msg = GuiMessage()
			msg.data = 'Something went wrong registering robot names and the subscriber listening to waypoint definitions may not have been enabled!!'
			msg.color = msg.COLOR_GRAY
			self.gui_message_pub.publish(msg)

	def recordWaypoint(self, msg):
		'''
		Record the movemment of the interactive marker
		'''

		self.waypoint = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

	
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
		self.waypoint_sub.unregister()
		
