#!/usr/bin/python

'''
Skeleton code to make a new plugin
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

from basestation_gui_python.msg import GuiMessage, DarpaStatus, ArtifactSubmissionReply

class ArtifactSubmissionPlugin(Plugin):

	submission_reply_trigger = pyqtSignal(object) #to keep the drawing on the proper thread

	def __init__(self, context):
		super(ArtifactSubmissionPlugin, self).__init__(context)
		self.setObjectName('MessagePlugin')		

		self.initPanel(context) #layout plugin

		#setup subscribers
		self.submission_reply_sub = rospy.Subscriber('/gui/submission_reply', ArtifactSubmissionReply, self.submissionReply)

		self.submission_reply_trigger.connect(self.submissionReplyMonitor)

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
		self.submission_reply_widget = QWidget()
		self.submission_reply_layout = qt.QGridLayout()

		submission_reply_label = qt.QLabel()
		submission_reply_label.setText('ARTIFACT SUBMISSION INFO')
		submission_reply_label.setAlignment(Qt.AlignCenter)
		self.submission_reply_layout.addWidget(submission_reply_label, 0, 0)

		#add the sort on/off button
		self.submission_reply_table_sort_button = qt.QPushButton("Sort by time")
		self.submission_reply_table_sort_button.setCheckable(True) # a button pressed will stay pressed, until unclicked
		self.submission_reply_table_sort_button.toggle() #start with it sorting the table

		self.submission_reply_layout.addWidget(self.submission_reply_table_sort_button, 1, 0)

		 #make a table
		self.submission_reply_table = qt.QTableWidget()


		#resize the cells to fill the widget 
		self.submission_reply_table.horizontalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)
		# self.arthist_table.verticalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)

		self.submission_reply_table.setColumnCount(4) # set column count        
		self.submission_reply_table.setHorizontalHeaderLabels(['Category', 'Time', 'x/y/z', 'Response']) #make the column headers

		#make sortable
		self.submission_reply_table.setSortingEnabled(True)

		tooltip_font = gui.QFont()
		tooltip_font.setPointSize(13)

		#add the table to the layout
		self.submission_reply_layout.addWidget(self.submission_reply_table)

		#add to the overall gui
		self.submission_reply_widget.setLayout(self.submission_reply_layout)
		self.global_widget.addWidget(self.submission_reply_widget)

	
	def submissionReply(self, msg):
		self.submission_reply_trigger.emit(msg)


	def submissionReplyMonitor(self, msg):
		'''
		Populate the table with infromation about an artifact that was submitted
		'''
		#check that threading is working properly
		if (not isinstance(threading.current_thread(), threading._MainThread)):
			print "Drawing on the submission history panel not guarented to be on the proper thread"	

		#make a table item to add 
		try:
			submission_time = self.displaySeconds(float(msg.submission_time_raw))
		except:
			submission_time = '99:99'

		if(msg.score_change==0):
			submission_correct = 'False'
			submission_color = gui.QColor(220,0,0)
		else:
			submission_correct = 'True'
			submission_color = gui.QColor(0,220,0)        

		response_item = qt.QTableWidgetItem('Info')
		response_item.setToolTip('DARPA response: ' + str(msg.report_status) + '\nHTTP Response: ' + \
								 str(msg.http_response) + str(msg.http_reason) + \
								 '\nSubmission Correct? ' + submission_correct)

		response_item.setBackground(submission_color)

		#add the item to the table
		self.submission_reply_table.setSortingEnabled(False)

		self.submission_reply_table.insertRow(self.submission_reply_table.rowCount())
		row = self.submission_reply_table.rowCount() - 1

		row_data = [msg.artifact_type, submission_time, str(int(msg.x))+'/'+\
														str(int(msg.y))+'/'+\
														str(int(msg.z))]

		for col, val in enumerate(row_data):            
			if (col==1):
				colon = submission_time.find(':')
				val = float(submission_time[:colon])*60 + float(submission_time[colon+1:])
				item = NumericItem(str(submission_time))
				item.setData(core.Qt.UserRole, val)

			else: #if we're not dealing with a display time
				item = NumericItem(str(val))
				item.setData(core.Qt.UserRole, val)

			item.setFlags( core.Qt.ItemIsSelectable |  core.Qt.ItemIsEnabled ) #make it non-editable
			item.setTextAlignment(Qt.AlignHCenter) 

			self.submission_reply_table.setItem(row, col, item)

		#add the response item (has tool-tip with info about the submission) we don't want this to be a NumericItem
		response_item.setFlags( core.Qt.ItemIsSelectable |  core.Qt.ItemIsEnabled )
		self.submission_reply_table.setItem(row, 3, response_item)

		self.submission_reply_table.setSortingEnabled(True)

		if(self.submission_reply_table_sort_button.isChecked()): #if the sort button is pressed, sort the incoming artifacts
			self.submission_reply_table.sortItems(1, core.Qt.DescendingOrder)		
			self.submission_reply_table.viewport().update()

	

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

			
	def shutdown_plugin(self):
		# TODO unregister all publishers here
		self.submission_reply_sub.unregister()

class NumericItem(qt.QTableWidgetItem):
	'''
	Class whic overwrites a pyqt table widget item in order to allow for better sorting (e.g. '2'<'100')
	'''
	def __lt__(self, other):
		return (self.data(core.Qt.UserRole) <\
				other.data(core.Qt.UserRole))
		
