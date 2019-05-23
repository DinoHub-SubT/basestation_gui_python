#!/usr/bin/python

'''
Plugin for displaying the artifact queue (table of collected artifacts)
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

from basestation_gui_python.msg import GuiMessage, DarpaStatus, Artifact, RadioMsg, ArtifactUpdate

class ArtifactQueuePlugin(Plugin):

	queue_trigger = pyqtSignal(object) #to keep the drawing on the proper thread
	archive_artifact_trigger = pyqtSignal() 
	update_table_trigger = pyqtSignal(object, object, object)
	submit_all_artifacts_trigger = pyqtSignal()
	remove_artifact_fom_queue_trigger = pyqtSignal(object)

	def __init__(self, context):
		super(ArtifactQueuePlugin, self).__init__(context)
		self.setObjectName('ArtifactQueuePlugin')	

		#get the robot names, artifact categories
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

		self.displayed_artifact_id = None

		self.initPanel(context) #layout plugin

		self.queue_trigger.connect(self.addArtifactToQueueMonitor)
		self.archive_artifact_trigger.connect(self.confirmArchiveArtifactMonitor)
		self.update_table_trigger.connect(self.updateQueueTableMonitor)
		self.submit_all_artifacts_trigger.connect(self.submitAllQueuedArtifactsMonitor)
		self.remove_artifact_fom_queue_trigger.connect(self.removeArtifactFromQueueMonitor)

		#setup subscribers/publishers
		self.add_new_artifact_pub = rospy.Publisher('/gui/generate_new_artifact_manual', Artifact, queue_size = 10) #manually generate a new artifact
		self.message_pub = rospy.Publisher('/gui/message_print', GuiMessage, queue_size=10) #print a message
		self.archive_artifact_pub = rospy.Publisher('/gui/archive_artifact', String, queue_size=10) #archive a message
		self.duplicate_pub = rospy.Publisher('/gui/duplicate_artifact', String, queue_size=10) #duplicate the message we're clicked on
		self.artifact_submit_pub = rospy.Publisher('/gui/submit_artifact_from_queue', String, queue_size=10) #submit artifacts from queue
		self.focus_on_artifact_pub = rospy.Publisher('/gui/focus_on_artifact', String, queue_size=10) #publish the artifact id we selected
		self.update_label_pub = rospy.Publisher('/gui/update_art_label', String, queue_size=10) #update image update panel to indicate change to artifact
		self.restart_manip_plugin_pub = rospy.Publisher('/gui/disable_confirm_cancel_manip_plugin', Bool, queue_size=10) #disable confirm/cancel sequence in manipulation plugin

		self.queue_sub = rospy.Subscriber('/gui/artifact_to_queue', Artifact, self.addArtifactToQueue) #some plugin wants us to add an artifact
		self.update_sub = rospy.Subscriber('/gui/update_artifact_in_queue', ArtifactUpdate, self.updateArtifactInQueue) # an artifact's property has changed. just update the queue accordingly
		self.remove_sub = rospy.Subscriber('/gui/remove_artifact_from_queue', String, self.removeArtifactFromQueue) #some plugin wants us to remove a plugin
		self.submit_from_manip_sub = rospy.Subscriber('/gui/submit_artifact_from_manip_plugin', String, self.artifactSubmitted) #the manipulation plugin has submitted an artifact, we need to keep
																																#track of this happening


	def initPanel(self, context):
		'''
		Initialize the panel for displaying widgets
		'''   

		#define the specific widget
		self.queue_widget = QWidget()
		self.queue_layout = qt.QGridLayout()

		context.add_widget(self.queue_widget)

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

		self.submit_all_queued_artifacts_button = qt.QPushButton("Submit all queued artifacts!")
		self.submit_all_queued_artifacts_button.clicked.connect(self.submitAllQueuedArtifacts)
		self.queue_layout.addWidget(self.submit_all_queued_artifacts_button, 3, 0, 1, 3)


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
		self.queue_layout.addWidget(self.queue_table, 4, 0, 1, 3)

		#add to the overall gui
		self.queue_widget.setLayout(self.queue_layout)

	

	############################################################################################
	# Functions dealing with updating artifacts in the queue
	############################################################################################

	def updateArtifactInQueue(self, msg):
		'''
		Find the element in the table we need to updsate. 

		msg is of type ArtifactUpdate containing the info to be changed
		'''

		artifact_row = None
		for row in range(self.queue_table.rowCount()):
			if (self.queue_table.item(row,5).text() == msg.unique_id):
				artifact_row = row
				break


		if (artifact_row != None):
			if (msg.update_type == ArtifactUpdate.PROPERTY_CATEGORY):
				self.updateQueueTable(row, 3, msg.category) 

			elif (msg.update_type == ArtifactUpdate.PROPERTY_PRIORITY):
				self.updateQueueTable(row, 1, msg.priority)

			else:
				update_msg = GuiMessage()
				update_msg.data = 'We received an update message of unknown type or of type pose. Artifact queue not updated'
				update_msg.color = update_msg.COLOR_RED
				self.message_pub.publish(update_msg)

		else: #we never found an artifact with the same id in the queue
			update_msg = GuiMessage()
			update_msg.data = 'Could not find artifact with proper unique_id in table. Artifact not updated in queue'
			update_msg.color = update_msg.COLOR_RED
			self.message_pub.publish(update_msg)


	def updateQueueTable(self, row, col, data):
		'''
		For proper threading. See function below. 
		'''
		self.update_table_trigger.emit(row, col, data)

	def updateQueueTableMonitor(self, row, col, data):
		'''
		Update an element in the queue table

		row, col define the coordinate of what we're updating. 
		data is the value (string) that we need to set that cell to
		'''

		#check that threading is working properly
		if (not isinstance(threading.current_thread(), threading._MainThread)):
			print "Drawing on the message panel not guarenteed to be on the proper thread"

		if (type(data) == str):
			self.queue_table.item(row,col).setText(data)
			self.queue_table.item(row,col).setBackground(gui.QColor(255,255,255)) # to rem0ve the green background if its the unread element

		else:
			msg = GuiMessage()
			msg.data = 'Tried to update queue table with non-string value. Therefore table not updated'
			msg.color = msg.COLOR_ORANGE
			self.message_pub.publish(msg)

	############################################################################################
	# Functions dealing with queue button presses
	############################################################################################


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
		Duplicate the artifact(s) that is being clicked on
		'''
		
		rows_selected = self.getSelectedRowIndices()

		if (len(rows_selected)==0):
			msg = GuiMessage()
			msg.data = 'No artifact(s) selected, did not duplicate anything'
			msg.color = msg.COLOR_ORANGE
			self.message_pub.publish(msg)

		#make the confirm/delete buttons pressable and the correct color
		else:
			msg = String()
			for row in rows_selected:
				msg.data = self.queue_table.item(row,5).text()
				self.duplicate_pub.publish(msg)

	def archiveArtifact(self):
		'''
		Archive artifact. Currently, this means it never shows back up
		in the gui. Adds it to a list of archived artifacts in the
		artifact handler
		'''

		#confirm we've actually selected an artifact
		rows_selected =  self.getSelectedRowIndices()


		if (len(rows_selected)==0):
			msg = GuiMessage()
			msg.data = 'No artifact(s) selected, did not delete anything'
			msg.color = msg.COLOR_ORANGE
			self.message_pub.publish(msg)

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
		For proper threading. See function below. 
		'''
		self.archive_artifact_trigger.emit()

	def confirmArchiveArtifactMonitor(self):
		'''
		A deletion of an artifact has been confirmed. Remove the artifact
		from the queue
		'''

		#check that threading is working properly
		if (not isinstance(threading.current_thread(), threading._MainThread)):
			print "Drawing on the message panel not guarenteed to be on the proper thread"

		rows_selected = self.getSelectedRowIndices()

		handler_msg = String()
		update_msg = GuiMessage()

		for row in sorted(rows_selected, reverse=True): #needs to in reverse order so we don't try to access a row that no longer exists

			#if we're archiving something we're viewing, reset the id we're viewing
			if (self.queue_table.item(row,5).text() == self.displayed_artifact_id):
				self.displayed_artifact_id = None 

			#delete it from the ArtifactHandler book-keeping
			handler_msg.data = self.queue_table.item(row,5).text()
			self.archive_artifact_pub.publish(handler_msg) #only send the unique ID
			
			#delete it from the queue table
			msg = String()
			msg.data = self.queue_table.item(row,5).text()
			self.removeArtifactFromQueue(msg)

			#reset the confirm/cancel buttons
			self.queue_archive_cancel_button.setStyleSheet("background-color:rgb(126, 126, 126)")
			self.queue_archive_cancel_button.setEnabled(False)

			self.queue_archive_confirm_button.setStyleSheet("background-color:rgb(126, 126, 126)")
			self.queue_archive_confirm_button.setEnabled(False)


	def cancelArchiveArtifact(self):
		'''
		Do not actually archive the artifact. Leave it as-is. 
		Just reset the confirm/cancel buttons
		'''
		self.queue_archive_cancel_button.setStyleSheet("background-color:rgb(126, 126, 126)")
		self.queue_archive_cancel_button.setEnabled(False)

		self.queue_archive_confirm_button.setStyleSheet("background-color:rgb(126, 126, 126)")
		self.queue_archive_confirm_button.setEnabled(False)

	def submitAllQueuedArtifacts(self):
		'''
		For threading properly. See function below.
		'''
		self.submit_all_artifacts_trigger.emit()

	def submitAllQueuedArtifactsMonitor(self):
		'''
		Submit all of the queued artifacts to DARPA
		'''

		#check that threading is working properly
		if (not isinstance(threading.current_thread(), threading._MainThread)):
			print "Drawing on the message panel not guarenteed to be on the proper thread"

		msg = String()

		for row in range(self.queue_table.rowCount()):
			msg.data = self.queue_table.item(row,5).text()
			self.artifact_submit_pub.publish(msg)

		#remove all of the rows from the table
		while (self.queue_table.rowCount() > 0):
			self.queue_table.removeRow(0)

	############################################################################################
	# Functions adding/removing artifacts from the queue
	############################################################################################

	def removeArtifactFromQueue(self, msg):
		'''
		For proper threading. See function below.
		'''
		self.remove_artifact_fom_queue_trigger.emit(msg)

	def removeArtifactFromQueueMonitor(self, msg):
		'''
		Remove an artifact from the queue, i.e. after is has been submitted

		msg is a string of the artifact unique_id to remove
		'''

		#check that threading is working properly
		if (not isinstance(threading.current_thread(), threading._MainThread)):
			print "Drawing on the message panel not guarenteed to be on the proper thread"

		if (msg.data == self.displayed_artifact_id):
			#we're trying to delete something we're currently viewing
			art_label_msg = String()
			art_label_msg.data = 'Artifact has been deleted!'
			self.update_label_pub.publish(art_label_msg)

		#find the artifact in the queue table
		for row in range(self.queue_table.rowCount()):
			# print self.queue_table.item(row,5).text(), msg.data

			if (self.queue_table.item(row,5).text() == msg.data):
				update_msg = GuiMessage()

				update_msg.data = 'Artifact removed from table:'+str(msg.data)
				update_msg.color = update_msg.COLOR_GREEN
				self.message_pub.publish(update_msg)
				
				#delete it from the queue table
				self.queue_table.removeRow(self.queue_table.item(row,0).row())

				return

		#if we get to this point, we did not find the artifact to delete
		update_msg = GuiMessage()
		update_msg.data = 'Could not find artifact with unique_id '+str(msg.data)+' in table. Artifact not removed from queue'
		update_msg.color = update_msg.COLOR_RED
		self.message_pub.publish(update_msg)

	
	def addArtifactToQueue(self, msg):
		'''
		For proper threading. See function below. 
		'''
		self.queue_trigger.emit(msg)


	def addArtifactToQueueMonitor(self, msg):
		'''
		Draw something on the gui in this function

		msg is a custom Artifact message containing info about the artifact
		'''

		#check that threading is working properly
		if (not isinstance(threading.current_thread(), threading._MainThread)):
			print "Drawing on the message panel not guarenteed to be on the proper thread"

		self.queue_table.setSortingEnabled(False) #to avoid corrupting the table

		self.queue_table.insertRow(self.queue_table.rowCount())
		row = self.queue_table.rowCount() - 1

		#generate the display time
		disp_time = self.displaySeconds(msg.time_from_robot)

		row_data = [msg.source_robot_id, msg.priority, disp_time, \
					msg.category, '!', msg.unique_id]


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


	############################################################################################
	# Misc. functions
	############################################################################################	

	def artifactSubmitted(self, msg):
		'''
		An artifact was just submitted. IF its the one we were viewing, reset the displayed artifact id

		msg is a String with the unique id of the artifact
		'''

		if (self.displayed_artifact_id == msg.data):
			self.displayed_artifact_id = None


	def queueClick(self, row, col):
		'''
		An artifact was clicked on the artifact queue

		row, col define the coordinates (ints) of the cells that was clicked on
		'''

		#select the whole row. for visualization mostly
		self.queue_table.selectRow(row)

		#remove the unread indicator from the last column
		self.updateQueueTable(row, 4, '')

		#so we know what's currently being displayed
		self.displayed_artifact_id = self.queue_table.item(row,5).text()

		#publish that we have selected this artifact
		msg = String()
		msg.data = self.queue_table.item(row,5).text() 
		self.focus_on_artifact_pub.publish(msg)

		#remove the artifact update message from the image visualizer plugin
		#if it is still visible
		msg = String()
		msg.data = 'hide'
		self.update_label_pub.publish(msg)

		#dectivate the proper buttons in the manipulation panel, we're viewing another artifact,
		#so restart the Confirm/Cancel sequence.
		self.restart_manip_plugin_pub.publish(Bool(True))
	
	def displaySeconds(self, seconds):
		'''
		Function to convert seconds float into a min:sec string

		seconds should be a float
		'''
		return str((int(float(seconds))/60))+':'+str(int(float(seconds)-(int(float(seconds))/60)*60))

	def resendArtifactInfo(self):
		'''
		Send a message to the robot to re-send of the artifacts it has detected.
		Useful if the robot goes out of comms range and then comes back in. We
		can then retreive the artifacts it detected when out of range.
		'''
		pass

	def shutdown_plugin(self):
		# TODO unregister all publishers here
		self.submit_from_manip_sub.unregister()
		self.queue_sub.unregister() 
		self.update_sub.unregister()
		self.remove_sub.unregister()


class NumericItem(qt.QTableWidgetItem):
	'''
	Class which overwrites a pyqt table widget item in order to allow for better sorting (e.g. '2'<'100')
	'''
	def __lt__(self, other):
		return (self.data(core.Qt.UserRole) <\
				other.data(core.Qt.UserRole))
