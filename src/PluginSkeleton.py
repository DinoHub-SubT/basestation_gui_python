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

from basestation_gui_python.msg import GuiMessage, DarpaStatus

class SkeletonPlugin(Plugin):

	skeleton_trigger = pyqtSignal(object) #to keep the drawing on the proper thread

	def __init__(self, context):
		super(SkeletonPlugin, self).__init__(context)
		self.setObjectName('SkeletonPlugin')		

		self.initPanel(context) #layout plugin

		#setup subscribers
		self.skeleton_sub = rospy.Subscriber('/gui_skeleton_print', GuiMessage, self.skeletonFunction)

		self.skeleton_trigger.connect(self.skeletonFunctionMonitor)

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
		self.skeleton_box_widget = QWidget()
		self.skeleton_box_layout = qt.QGridLayout()

		skeleton_label = qt.QLabel()
		skeleton_label.setText('Skeleton PANEL')
		skeleton_label.setAlignment(Qt.AlignCenter)
		self.skeleton_box_layout.addWidget(skeleton_label, 0, 0)

		#add to the overall gui
		self.skeleton_box_widget.setLayout(self.skeleton_box_layout)
		self.global_widget.addWidget(self.skeleton_box_widget)

	
	def skeletonFunction(self, msg):
		self.skeleton_trigger.emit(msg)


	def skeletonFunctionMonitor(self, msg):
		'''
		Draw something on the gui in this function
		'''
		#check that threading is working properly
		if (not isinstance(threading.current_thread(), threading._MainThread)):
			print "Drawing on the message panel not guarented to be on the proper thread"			

			
	def shutdown_plugin(self):
		# TODO unregister all publishers here
		pass
		
