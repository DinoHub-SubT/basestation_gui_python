#!/usr/bin/env python

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

from python_qt_binding.QtCore import Slot, Qt, qVersion, qWarning, Signal
from python_qt_binding.QtGui import QColor, QPixmap
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy

import pdb
from PyQt5.QtCore import pyqtSignal

from basestation_gui_python.msg import GuiMessage, DarpaStatus
from gui_utils import displaySeconds

class InfoPlugin(Plugin):

	info_trigger = pyqtSignal(object) #to keep the message printing on the proper thread

	def __init__(self, context):
		super(InfoPlugin, self).__init__(context)
		self.setObjectName('InfoPlugin')		

		self.initPanel(context) #layout plugin

		self.info_trigger.connect(self.setInfoStringMonitor)

		#setup subscribers
		self.info_string = None
		self.info_sub = rospy.Subscriber('/gui/darpa_status', DarpaStatus, self.setInfoString) #for gathering info from darpa

		

	def initPanel(self, context):
		'''
		Initialize the panel for displaying widgets
		'''

		#define the overall widget
		self.info_box_widget = QWidget()
		self.info_box_layout = qt.QGridLayout()

		context.add_widget(self.info_box_widget)

		bold_font = gui.QFont() 
		bold_font.setBold(True)
		bold_font.setPointSize(16)

		self.info_label = qt.QLabel()
		self.info_label.setText('Time: -- \t Score: -- \t Proposals Left: --')
		self.info_label.setAlignment(Qt.AlignCenter)
		self.info_label.setFont(bold_font)
		self.info_label.setStyleSheet('border:3px solid rgb(0, 0, 0);')
		self.info_box_layout.addWidget(self.info_label, 1, 0)


		#add to the overall gui
		self.info_box_widget.setLayout(self.info_box_layout)

	def setInfoString(self, msg):
		'''
		For proper threading. See function below.
		'''
		self.info_trigger.emit(msg)


	def setInfoStringMonitor(self, msg):
		'''
		Update the information string displaying darpa information

		msg is a custom DarpaStatus msg containing info about the time_elpased, score
		and remaining reports as received from DARPA
		'''

		#check that threading is working properly
		if (not isinstance(threading.current_thread(), threading._MainThread)):
			print "Drawing on the message panel not guarented to be on the proper thread"	

		self.info_label.setText("Time: "+str(displaySeconds(msg.time_elapsed))+'\t Score: '+\
								str(msg.score)+ '\t Remaining Reports: '+\
								str(msg.remaining_reports))

	

			
	def shutdown_plugin(self):
		# TODO unregister all publishers here
		self.info_sub.unregister()
		
