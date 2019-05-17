#!/usr/bin/python

'''
Plugin for displaying the artifact queue (table of collected artifacts)
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
from GuiEngine import GuiEngine, Artifact
from PyQt5.QtCore import pyqtSignal

from basestation_gui_python.msg import GuiMessage, DarpaStatus, Artifact

class ArtifactQueuePlugin(Plugin):

	queue_trigger = pyqtSignal(object) #to keep the drawing on the proper thread

	def __init__(self, context):
		super(ArtifactQueuePlugin, self).__init__(context)
		self.setObjectName('ArtifactQueuePlugin')	

		#get the robot names, artifact categories	

		#get the number of robots
		rospack = rospkg.RosPack()
		config_filename = rospack.get_path('basestation_gui_python')+'/config/gui_params.yaml'
		config = yaml.load(open(config_filename, 'r').read())

		self.robot_names, self.artifact_categories = [], []

		exp_params = config['experiment_params']
		
		for name in exp_params['robot_names']:
			self.robot_names.append(name)

		# read info on darpa-related commands (communication protocol, etc.)
		darpa_params = config['darpa_params']

		#artifact categories used for the gui
		self.artifact_categories = []
		for category in darpa_params['artifact_categories']:
			self.artifact_categories.append(category)

		self.initPanel(context) #layout plugin

		#setup subscribers
		self.queue_sub = rospy.Subscriber('/gui/artifact_to_queue', Artifact, self.addArtifactToQueue)

		self.add_new_artifact_pub = rospy.Publisher('/gui/generate_new_artifact', Artifact, queue_size = 10)
		self.message_pub = rospy.Publisher('/gui/message_print', GuiMessage, queue_size=10)

		self.queue_trigger.connect(self.addArtifactToQueueMonitor)

	def initPanel(self, context):
		'''
		Initialize the panel for displaying widgets
		'''   

		#define the overall plugin
		self.widget = QWidget()
		self.global_widget = qt.QGridLayout() 

		self.widget.setLayout(self.global_widget)
		context.add_widget(self.widget)

		#define the specific widget
		self.queue_widget = QWidget()
		self.queue_layout = qt.QGridLayout()

		queue_label = qt.QLabel()
		queue_label.setText('ARTIFACT QUEUE')
		queue_label.setAlignment(Qt.AlignCenter)
		self.queue_layout.addWidget(queue_label, 0, 0, 1, 3)

		#add the sort on/off button
		self.queue_table_sort_button = qt.QPushButton("Sort by time")
		self.queue_table_sort_button.setCheckable(True) # a button pressed will stay pressed, until unclicked
		self.queue_table_sort_button.toggle() #start with it sorting the table
		self.queue_layout.addWidget(self.queue_table_sort_button, 1, 0)

		#add the insert new artifact button
		self.queue_insert_artifact_button = qt.QPushButton("Add artifact")
		self.queue_insert_artifact_button.clicked.connect(self.manuallyAddArtifact)
		self.queue_layout.addWidget(self.queue_insert_artifact_button, 1, 1)

		#add the duplicate new artifact button
		self.queue_duplicate_artifact_button = qt.QPushButton("Duplicate artifact")
		self.queue_duplicate_artifact_button.clicked.connect(self.duplicateArtifact)
		self.queue_layout.addWidget(self.queue_duplicate_artifact_button, 1, 2)

		#add support for deleting artifacts
		self.queue_archive_artifact_button = qt.QPushButton("Archive artifact")
		self.queue_archive_artifact_button.clicked.connect(self.archiveArtifact)
		self.queue_layout.addWidget(self.queue_archive_artifact_button, 2, 0)

		self.queue_archive_confirm_button = qt.QPushButton("Confirm")
		self.queue_archive_confirm_button.setCheckable(True) # a button pressed will stay pressed, until unclicked
		self.queue_archive_confirm_button.clicked.connect(self.confirmArchiveArtifact)
		self.queue_archive_confirm_button.setStyleSheet("background-color:rgb(126, 126, 126)")
		self.queue_archive_confirm_button.setEnabled(False)
		self.queue_layout.addWidget(self.queue_archive_confirm_button, 2, 2)

		self.queue_archive_cancel_button = qt.QPushButton("Cancel")
		self.queue_archive_cancel_button.setCheckable(True) # a button pressed will stay pressed, until unclicked
		self.queue_archive_cancel_button.clicked.connect(self.cancelArchiveArtifact)
		self.queue_archive_cancel_button.setStyleSheet("background-color:rgb(126, 126, 126)")
		self.queue_archive_cancel_button.setEnabled(False)
		self.queue_layout.addWidget(self.queue_archive_cancel_button, 2, 1)

		#add support for requesting that all artifact info gets sent back
		self.resend_art_data_button = qt.QPushButton("Re-send all artifact info")
		self.resend_art_data_button.clicked.connect(self.resendArtifactInfo)
		self.queue_layout.addWidget(self.resend_art_data_button, 3, 0, 1, 3)

		self.submit_all_queued_artifacts_button = qt.QPushButton("Submit all queued artifacts!")
		self.submit_all_queued_artifacts_button.clicked.connect(self.submitAllQueuedArtifacts)
		self.queue_layout.addWidget(self.submit_all_queued_artifacts_button, 4, 0, 1, 3)


		#make a table
		self.queue_table = qt.QTableWidget()

		#resize the cells to fill the widget 
		self.queue_table.horizontalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)

		self.queue_table.setColumnCount(6) # set column count        
		self.queue_table.setHorizontalHeaderLabels(['Robot\nNum', 'Priority', 'Detect\nTime', '   Category   ', 'Unread', 'Unique ID']) #make the column headers

		
		self.queue_table.setColumnHidden(5,True) #hide the unique_id    
		self.queue_table.setSortingEnabled(True) #make sortable

		#add click listener
		self.queue_table.cellClicked.connect(self.queueClick)

		#add the table to the layout
		self.queue_layout.addWidget(self.queue_table, 5, 0, 1, 3)

		#add to the overall gui
		self.queue_widget.setLayout(self.queue_layout)
		self.global_widget.addWidget(self.queue_widget) #last 2 parameters are rowspan and columnspan


	def manuallyAddArtifact(self):
		'''
		For BSM to manually add an artifact
		'''

		msg = Artifact()
		msg.original_timestamp = -1
		msg.category = self.artifact_categories[0]
		msg.orig_pose.position.x = -0.1
		msg.orig_pose.position.y = -0.1
		msg.orig_pose.position.z = -0.1
		msg.source_robot_id = -1
		msg.artifact_report_id = -1

		self.add_new_artifact_pub.publish(msg)

	def duplicateArtifact(self):
		'''
		Duplicate the artifact that is being clicked on
		'''
		pass

	def archiveArtifact(self):
		'''
		Archive artifact. At this moment, this means it never shows back up
		in the gui. Adds it to a list of arhcived artifacts in the
		artifact handler
		'''

		#confirm we've actually selected an artifact
		rows_selected =  self.getSelectedRowIndices()


		if (len(rows_selected)==0):
			msg = GuiMessage()
			msg.data = 'No artifact(s) selected, did not delete anything'
			msg.color = msg.COLOR_ORANGE
			self.message_pub.publish(msg)
			print "here"

		#make the confirm/delete buttons pressable and the correct color
		else:
			self.queue_archive_cancel_button.setStyleSheet("background-color:rgb(220, 0, 0)")
			self.queue_archive_cancel_button.setEnabled(True)

			self.queue_archive_confirm_button.setStyleSheet("background-color:rgb(0, 220, 0)")
			self.queue_archive_confirm_button.setEnabled(True) 


	def getSelectedRowIndices(self):
		'''
		Returns the indice(s) of the selected rows in the queue table.
		Always returns a list, unlike self.queue_table.selectionModel().selectedRows()
		'''

		if(self.queue_table.rowCount()==0):
			return []

		return [a.row() for a in self.queue_table.selectionModel().selectedRows()]



	def confirmArchiveArtifact(self):
		'''
		A deletion of am artifact has been confirmed. Remove the artifact
		from the queue
		'''
		rows_selected = self.getSelectedRowIndices()

		for row in rows_selected:
			r=0

	def cancelArchiveArtifact(self):
		'''
		Do not actually archive the artifact. Leave it as-is. 
		'''
		pass

	def resendArtifactInfo(self):
		'''
		Send a message to the robot to re-send of the artifacts it has detected.
		Useful if the robot goes out of comms range and then comes back in. We
		can then retreive the artifacts it detected when out of range.
		'''
		pass

	def submitAllQueuedArtifacts(self):
		'''
		Submit all of the queued artifacts to DARPA
		'''

		#PROBABLY PUT A CONFIRM/DELETE BUTTON HERE
		pass

	def queueClick(self, row, col):
		'''
		An artifact was clicked on the artifact queue
		'''

		#select the whole row. for visualization mostly
		self.queue_table.selectRow(row)

	def displaySeconds(self, seconds):
		'''
		Function to convert seconds float into a min:sec string
		'''
		return str((int(float(seconds))/60))+':'+str(int(float(seconds)-(int(float(seconds))/60)*60))



	
	def addArtifactToQueue(self, msg):
		self.queue_trigger.emit(msg)


	def addArtifactToQueueMonitor(self, msg):
		'''
		Draw something on the gui in this function
		'''
		#check that threading is working properly
		if (not isinstance(threading.current_thread(), threading._MainThread)):
			print "Drawing on the message panel not guarented to be on the proper thread"

		self.queue_table.setSortingEnabled(False) #to avoid corrupting the table

		self.queue_table.insertRow(self.queue_table.rowCount())
		row = self.queue_table.rowCount() - 1

		#generate the display time
		disp_time = self.displaySeconds(msg.time_from_robot)

		row_data = [msg.source_robot_id, msg.priority, disp_time, \
					msg.category, '!', msg.unique_id]

		print row_data

		for col, val in enumerate(row_data):

			if (col==2):
				colon = disp_time.find(':')
				val = float(disp_time[:colon])*60 + float(disp_time[colon+1:])
				item = NumericItem(str(disp_time))
				item.setData(core.Qt.UserRole, val)
			
			else: #if we're not dealing with a display time
				item = NumericItem(str(val))
				item.setData(core.Qt.UserRole, val)                    

			self.queue_table.setItem(row, col, item)

		#color the unread green
		self.queue_table.item(row, 4).setBackground(gui.QColor(0,255,0))

		for i in range(self.queue_table.columnCount()): #make the cells not editable and make the text centered
			if self.queue_table.item(row, i) != None: 
				self.queue_table.item(row, i).setFlags( core.Qt.ItemIsSelectable |  core.Qt.ItemIsEnabled )
				self.queue_table.item(row, i).setTextAlignment(Qt.AlignHCenter) 

		self.queue_table.setSortingEnabled(True) #to avoid corrupting the table

		if(self.queue_table_sort_button.isChecked()): #if the sort button is pressed, sort the incoming artifacts
			self.queue_table.sortItems(2, core.Qt.DescendingOrder)
			
			self.queue_table.viewport().update()			

			
	def shutdown_plugin(self):
		# TODO unregister all publishers here
		pass

class NumericItem(qt.QTableWidgetItem):
	'''
	Class which overwrites a pyqt table widget item in order to allow for better sorting (e.g. '2'<'100')
	'''
	def __lt__(self, other):
		return (self.data(core.Qt.UserRole) <\
				other.data(core.Qt.UserRole))
