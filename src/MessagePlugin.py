#!/usr/bin/python

'''
Plugin to display text messages about various going-ons
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

from python_qt_binding.QtCore import Slot, Qt, qVersion, qWarning, Signal
from python_qt_binding.QtGui import QColor, QPixmap
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy

import pdb
from PyQt5.QtCore import pyqtSignal

from basestation_gui_python.msg import GuiMessage, DarpaStatus

class MessagePlugin(Plugin):

	print_message_trigger = pyqtSignal(object) #to keep the message printing on the proper thread

	def __init__(self, context):
		super(MessagePlugin, self).__init__(context)
		self.setObjectName('MessagePlugin')		

		self.darpa_time = None	

		self.initMessagePanel(context) #layout plugin

		self.print_message_trigger.connect(self.printMessageMonitor) #to print in a thread-safe manner

		#setup subscribers
		self.message_sub = rospy.Subscriber('/gui/message_print', GuiMessage, self.printMessage) #contains message data to disp
		self.time_sub    = rospy.Subscriber('/gui/darpa_status', DarpaStatus, self.setDarpaTime) #contains darpa status information

		
	def initMessagePanel(self, context):
		'''
		Initialize the panel which displays messages
		'''

		#define the overall widget
		self.message_box_widget = QWidget()
		self.message_box_layout = qt.QGridLayout()

		context.add_widget(self.message_box_widget)

		message_label = qt.QLabel()
		message_label.setText('MESSAGE PANEL')
		message_label.setAlignment(Qt.AlignCenter)
		self.message_box_layout.addWidget(message_label, 0, 0)


		self.message_textbox = qt.QListWidget()
		self.message_textbox.setWordWrap(True)

		self.message_box_layout.addWidget(self.message_textbox, 1, 0)


		#add to the overall gui
		self.message_box_widget.setLayout(self.message_box_layout)

	def setDarpaTime(self, msg):
		'''
		Function that saves the darpa time published from the darpa status node
		into a local variable to be used here

		msg is a custom DarpaStatus message which contains info about time/score/remaining reports
		'''
		self.darpa_time = msg.time_elapsed

	def printMessage(self, msg):
		'''
		For being thread-safe. See function below
		'''
		self.print_message_trigger.emit(msg)


	def printMessageMonitor(self, msg):
		'''
		Add message to the message box that is simply a string from
		this application (not ROS)

		msg is a custom GuiMessage msg which contains a field for text and 
			constants for setting message color, depnding on importance of message
		'''

		#check that threading is working properly
		if (not isinstance(threading.current_thread(), threading._MainThread)):
			print "Drawing on the message panel not guarented to be on the proper thread"		
		

		if (self.darpa_time != None):
			item = qt.QListWidgetItem('['+self.displaySeconds(self.darpa_time)+']'+msg.data)
		
		else: #if we're not connect to the command post, don't display a time
			item = qt.QListWidgetItem('[--] '+msg.data)

		if msg.color == msg.COLOR_ORANGE:
			msg_color = [242., 143, 50.]
		elif msg.color == msg.COLOR_RED:
			msg_color = [250,128,114]
		elif msg.color == msg.COLOR_GREEN:
			msg_color = [144,238,144]
		elif msg.color == msg.COLOR_GRAY:
			msg_color = [220,220,220]
		else:
			msg_color = [255, 255, 255]


		item.setBackground(gui.QColor(msg_color[0], msg_color[1], msg_color[2]))

		self.message_textbox.addItem(item)
		self.message_textbox.sortItems(core.Qt.DescendingOrder)

		self.message_textbox.viewport().update() #refresh the message box

	def displaySeconds(self, seconds):
		'''
		Function to convert seconds float into a min:sec string

		seconds should be a float. typically from DARPA of the elapsed time of the run
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
		# TODO unregister all subscribers here
		self.message_sub.unregister()
		self.time_sub.unregister()
		
