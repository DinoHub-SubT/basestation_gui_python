#!/usr/bin/python

'''
Plugin to display status information from the robot
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebation or Matt).
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
from GuiEngine import GuiEngine, Artifact
from PyQt5.QtCore import pyqtSignal

from basestation_gui_python.msg import GuiMessage, DarpaStatus, StatusPanelUpdate
import yaml

class RobotStatusPlugin(Plugin):

	robot_status_trigger = pyqtSignal(object) #to keep the drawing on the proper thread

	def __init__(self, context):
		super(RobotStatusPlugin, self).__init__(context)
		self.setObjectName('RobotStatusPlugin')	

		#get the number of robots
		rospack = rospkg.RosPack()
		config_filename = rospack.get_path('basestation_gui_python')+'/config/gui_params.yaml'
		config = yaml.load(open(config_filename, 'r').read())

		self.robot_names = []
		exp_params = config['experiment_params']
		for name in exp_params['robot_names']:
			self.robot_names.append(name)	

		#define the rows of the table
		self.status = ['Battery(mins)', 'Comms', 'Mobility', 'CPU', 'Disk Space'] #define the status each robot will have

		self.initPanel(context) #layout plugin

		#setup subscribers
		self.robot_status_sub = rospy.Subscriber('/status_panel_update', StatusPanelUpdate, self.robotStatus)

		self.robot_status_trigger.connect(self.robotStatusMonitor)

	def initPanel(self, context):
		'''
		Initialize the panel for displaying widgets
		'''

		#define the overall plugin
		self.widget = QWidget()
		self.global_widget = qt.QGridLayout()     
		
		self.widget.setLayout(self.global_widget)
		context.add_widget(self.widget)

		#define the overall widget
		self.robot_status_widget = QWidget()
		self.robot_status_layout = qt.QGridLayout()

		num_robots = len(self.robot_names) #get the number of robots
		statuses = ['Battery(mins)', 'Comms', 'Mobility', 'CPU', 'Disk Space'] #define the status each robot will have
		self.statuses = statuses

		status_label = qt.QLabel()
		status_label.setText('STATUS PANEL')
		status_label.setAlignment(Qt.AlignCenter)
		self.robot_status_layout.addWidget(status_label, 0,0)

		#make a table
		self.status_table = qt.QTableWidget()

		#resize the cells to fill the widget 
		self.status_table.horizontalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)
		self.status_table.verticalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)
		
		self.status_table.setRowCount(len(statuses)) # set row count
		self.status_table.setColumnCount(num_robots) # set column count

		#make the row and column headers
		self.status_table.setVerticalHeaderLabels(statuses) 
		self.status_table.setHorizontalHeaderLabels(self.robot_names) 

		#add the table to the layout
		self.robot_status_layout.addWidget(self.status_table, 1, 0)

		#add to the overall guirobot_status_layout
		self.robot_status_widget.setLayout(self.robot_status_layout)
		self.global_widget.addWidget(self.robot_status_widget)

	
	def robotStatus(self, msg):
		self.robot_status_trigger.emit(msg)


	def robotStatusMonitor(self, msg):
		'''
		Draw something on the gui in this function
		'''
		#check that threading is working properly
		if (not isinstance(threading.current_thread(), threading._MainThread)):
			print "Drawing on the message panel not guarented to be on the proper thread"	

		msg_str = [ord(c) for c in msg.value]
		column_index = msg.robot_id

		if column_index < 0 or column_index >= self.status_table.columnCount():
			rospy.logerr('robot_id %d is out of bounds: ' % column_index)
			return

		if msg.key not in self.statuses:
			rospy.logerr('key %s is not the name of a row in the status panel' % msg.key)
			return

		row_index = self.statuses.index(msg.key)

		item = self.status_table.item(row_index, column_index)
		
		if item == None:
			self.status_table.setItem(row_index, column_index, qt.QTableWidgetItem(''))
		item = self.status_table.item(row_index, column_index)
		item.setBackground(gui.QColor(msg.color.r, msg.color.g, msg.color.b))

		item.setText(msg.value)
		
		self.status_table.viewport().update()		

			
	def shutdown_plugin(self):
		# TODO unregister all publishers here
		pass
		
