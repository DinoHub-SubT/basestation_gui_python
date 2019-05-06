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

from basestation_gui_python.msg import GuiMessage

class MessagePlugin(Plugin):

	print_message_trigger = pyqtSignal(object) #to keep the message printing on the proper thread

	def __init__(self, context):
		super(MessagePlugin, self).__init__(context)
		self.setObjectName('MessagePlugin')		

		self.initMessagePanel(context) #layout plugin

		#setup subscribers
		self.message_sub = rospy.Subscriber('/gui_message_print', GuiMessage, self.printMessage)

		self.print_message_trigger.connect(self.printMessageMonitor)

	def initMessagePanel(self, context):
		'''
		Initialize the panel which displays various text messages
		'''

		#define the overall plugin
		self.widget = QWidget()
		self.global_widget = qt.QGridLayout()     
		
		self.widget.setLayout(self.global_widget)
		context.add_widget(self.widget)

		#define the overall widget
		self.message_box_widget = QWidget()
		self.message_box_layout = qt.QGridLayout()

		message_label = qt.QLabel()
		message_label.setText('MESSAGE PANEL')
		message_label.setAlignment(Qt.AlignCenter)
		self.message_box_layout.addWidget(message_label, 0, 0)


		self.message_textbox = qt.QListWidget()
		self.message_textbox.setWordWrap(True)

		self.message_box_layout.addWidget(self.message_textbox, 1, 0)


		#add to the overall gui
		self.message_box_widget.setLayout(self.message_box_layout)
		self.global_widget.addWidget(self.message_box_widget)


	def printMessage(self, msg):
		self.print_message_trigger.emit(msg)


	def printMessageMonitor(self, msg):
		'''
		Add message to the message box that is simply a string from
		this application (not ROS)
		'''

		#check that threading is working properly
		if (not isinstance(threading.current_thread(), threading._MainThread)):
			print "Drawing on the message panel not guarented to be on the proper thread"		
		
		item = qt.QListWidgetItem('[--] '+msg.data)

		if (msg.color.r == 0) and (msg.color.g == 0) and (msg.color.b == 0): #if color hasn't been set, set to white
			item.setBackground(gui.QColor(255, 255, 255))
		else: #set to color of message
			item.setBackground(gui.QColor(msg.color.r, msg.color.b, msg.color.g))

		self.message_textbox.addItem(item)

		self.message_textbox.viewport().update()


	def select_config_file(self):
		starting_path = os.path.join(rospkg.RosPack().get_path('basestation_gui_python'), 'config')
		filename = qt.QFileDialog.getOpenFileName(self.widget, 'Open Config File', starting_path, "Config Files (*.yaml)")[0]
		if filename != '':
			self.initiateSettings(filename)
			
	def shutdown_plugin(self):
		# TODO unregister all publishers here
		self.message_sub.unregister()
		
