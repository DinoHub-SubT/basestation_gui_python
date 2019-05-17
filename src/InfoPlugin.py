#!/usr/bin/python

'''
The info panel in the gui (which displays time remaining, score, remaining reports)
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

from basestation_gui_python.msg import GuiMessage, DarpaStatus

class InfoPlugin(Plugin):

	info_trigger = pyqtSignal(object) #to keep the message printing on the proper thread

	def __init__(self, context):
		super(InfoPlugin, self).__init__(context)
		self.setObjectName('InfoPlugin')		

		self.initPanel(context) #layout plugin

		#setup subscribers
		self.info_string = None
		self.info_sub = rospy.Subscriber('/gui/darpa_status', DarpaStatus, self.setInfoString)

		self.info_trigger.connect(self.setInfoStringMonitor)

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
		self.info_box_widget = QWidget()
		self.info_box_layout = qt.QGridLayout()

		boldFont = gui.QFont()
		boldFont.setBold(True)

		self.info_label = qt.QLabel()
		self.info_label.setText('Time: -- \t Score: -- \t Proposals Left: --')
		self.info_label.setAlignment(Qt.AlignCenter)
		self.info_label.setFont(boldFont)
		self.info_label.setStyleSheet('border:3px solid rgb(0, 0, 0);')
		self.info_box_layout.addWidget(self.info_label, 1, 0)


		#add to the overall gui
		self.info_box_widget.setLayout(self.info_box_layout)
		self.global_widget.addWidget(self.info_box_widget)

	def setInfoString(self, msg):
		self.info_trigger.emit(msg)


	def setInfoStringMonitor(self, msg):
		'''
		Draw something on the gui in this function
		'''
		#check that threading is working properly
		if (not isinstance(threading.current_thread(), threading._MainThread)):
			print "Drawing on the message panel not guarented to be on the proper thread"	

		self.info_label.setText("Time: "+str(self.displaySeconds(msg.time_remaining))+'\t Score: '+\
								str(msg.score)+ '\t Remaining Reports: '+\
								str(msg.remaining_reports))

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
		self.info_sub.unregister()
		
