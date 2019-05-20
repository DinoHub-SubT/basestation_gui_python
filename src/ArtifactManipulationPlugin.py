#!/usr/bin/python

'''
RQT Plugin for inspecting artifacts (looking at images, positions, etc.)
and changing artifact properties (position, category, etc.)
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

class ArtifactManipulationPlugin(Plugin):

	focus_on_artifact_trigger = pyqtSignal(object) #to keep the drawing on the proper thread

	def __init__(self, context):
		super(ArtifactManipulationPlugin, self).__init__(context)
		self.setObjectName('ArtifactManipulationPlugin')

		#get the artifact categories
		rospack = rospkg.RosPack()
		config_filename = rospack.get_path('basestation_gui_python')+'/config/gui_params.yaml'
		config = yaml.load(open(config_filename, 'r').read())

		self.artifact_categories = []

		exp_params = config['darpa_params']
		
		for name in exp_params['artifact_categories']:
			self.artifact_categories.append(name)

		self.initPanel(context) #layout plugin

		#setup subscribers
		self.focus_on_artifact_sub = rospy.Subscriber('/gui/focus_on_artifact', String, self.focusOnArtifact)

		self.focus_on_artifact_trigger.connect(self.focusOnArtifactMonitor)

	def initPanel(self, context):
		'''
		Initialize the panel for displaying widgets
		'''

		#define the overall plugin
		self.widget = QWidget()
		self.global_widget = qt.QGridLayout() 

		self.widget.setLayout(self.global_widget)
		context.add_widget(self.widget) 

		# define the overall widget
		self.artmanip_widget = QWidget()
		self.artmanip_layout = qt.QGridLayout() 

		boldFont = gui.QFont()
		boldFont.setBold(True)              

		#add in information about 3d position   
		robot_pos = ['N/A', 'N/A', 'N/A'] #fake data

		self.orig_pos_label = qt.QLabel()
		self.orig_pos_label.setText('\n\nOriginal Position (XYZ)')
		self.orig_pos_label.setAlignment(Qt.AlignCenter)
		self.orig_pos_label.setFont(boldFont)
		self.artmanip_layout.addWidget(self.orig_pos_label, 1, 0, 1, 3)

		self.orig_pos_label_x = qt.QLabel()
		self.orig_pos_label_x.setText(str(robot_pos[0]))
		self.orig_pos_label_x.setAlignment(Qt.AlignCenter)
		self.artmanip_layout.addWidget(self.orig_pos_label_x, 2, 0)

		self.orig_pos_label_y = qt.QLabel()
		self.orig_pos_label_y.setText(str(robot_pos[1]))
		self.orig_pos_label_y.setAlignment(Qt.AlignCenter)
		self.artmanip_layout.addWidget(self.orig_pos_label_y, 2, 1)

		self.orig_pos_label_z = qt.QLabel()
		self.orig_pos_label_z.setText(str(robot_pos[2]))
		self.orig_pos_label_z.setAlignment(Qt.AlignCenter)
		self.artmanip_layout.addWidget(self.orig_pos_label_z, 2, 2)

		#editable information about the position, to send to darpa
		self.refined_pos_label = qt.QLabel()
		self.refined_pos_label.setText('\nRefined Position (XYZ)')
		self.refined_pos_label.setFont(boldFont)
		self.refined_pos_label.setAlignment(Qt.AlignCenter)
		self.artmanip_layout.addWidget(self.refined_pos_label, 3, 0, 1, 3)

		self.art_pos_textbox_x, self.art_pos_textbox_y, self.art_pos_textbox_z = qt.QLineEdit(), qt.QLineEdit(), qt.QLineEdit()
	   
		#fill in some fake data
		self.art_pos_textbox_x.setText(str(robot_pos[0]))
		self.artmanip_layout.addWidget(self.art_pos_textbox_x, 4, 0)

		self.art_pos_textbox_y.setText(str(robot_pos[1]))
		self.artmanip_layout.addWidget(self.art_pos_textbox_y, 4, 1)

		self.art_pos_textbox_z.setText(str(robot_pos[2]))
		self.artmanip_layout.addWidget(self.art_pos_textbox_z, 4, 2)

		#add button for displaying the interactive marker
		self.art_refinement_button = qt.QPushButton("Show Refinement Marker")
		self.art_refinement_button.setCheckable(True)
		self.art_refinement_button.clicked.connect(self.processArtRefinementPress)
		self.artmanip_layout.addWidget(self.art_refinement_button, 5, 0 , 1, 3)


		#add in a few buttons at the bottom to do various things
		self.artmanip_button_layout = qt.QHBoxLayout()
		self.artmanip_button_list = []

		art_action_label2 = qt.QLabel()
		art_action_label2.setText('\n\nCategory')
		art_action_label2.setFont(boldFont)
		art_action_label2.setAlignment(Qt.AlignCenter)
		self.artmanip_layout.addWidget(art_action_label2, 6, 0, 1, 2)

		art_action_label3 = qt.QLabel()
		art_action_label3.setText('\n\nPriority')
		art_action_label3.setFont(boldFont)
		art_action_label3.setAlignment(Qt.AlignCenter)
		self.artmanip_layout.addWidget(art_action_label3, 6, 2)

		 #make the combobox for setting the artifact category
		self.darpa_cat_box = qt.QComboBox()
		
		for category in self.artifact_categories:
			self.darpa_cat_box.addItem(category)
		
		self.darpa_cat_box.currentTextChanged.connect(self.updateArtifactCat)

		self.artmanip_layout.addWidget(self.darpa_cat_box, 7, 0, 1, 2)

		#make the combobox for setting the artifact priority
		self.artifact_priority_box = qt.QComboBox() 

		self.artifact_priority_box.addItem('High')
		self.artifact_priority_box.addItem('Med')
		self.artifact_priority_box.addItem('Low')
		
		self.artifact_priority_box.currentTextChanged.connect(self.updateArtifactPriority)
		self.artmanip_layout.addWidget(self.artifact_priority_box, 7, 2)


		self.darpa_button = qt.QPushButton("To DARPA")
		self.darpa_button.clicked.connect(partial(self.decideArtifact))
		self.darpa_button.setStyleSheet("background-color:rgb(255,130,0)")
		self.artmanip_layout.addWidget(self.darpa_button, 8, 0, 1, 3)

		self.darpa_confirm_button = qt.QPushButton("Confirm")
		self.darpa_confirm_button.clicked.connect(partial(self.proposeArtifact))
		self.darpa_confirm_button.setStyleSheet("background-color:rgb(100, 100, 100)")
		self.darpa_confirm_button.setEnabled(False)
		self.artmanip_layout.addWidget(self.darpa_confirm_button, 9, 2)

		self.darpa_cancel_button = qt.QPushButton("Cancel")
		self.darpa_cancel_button.clicked.connect(partial(self.cancelProposal))
		self.darpa_cancel_button.setStyleSheet("background-color:rgb(100, 100, 100)")
		self.darpa_cancel_button.setEnabled(False)
		self.artmanip_layout.addWidget(self.darpa_cancel_button, 9, 0)		

		#add to the overall gui
		self.artmanip_widget.setLayout(self.artmanip_layout)
		self.global_widget.addWidget(self.artmanip_widget)

	def processArtRefinementPress(self):
		pass
	def updateArtifactCat(self):
		pass
	def updateArtifactPriority(self):
		pass
	def decideArtifact(self):
		pass
	def proposeArtifact(self):
		pass
	def cancelProposal(self):
		pass

	
	def focusOnArtifact(self, msg):
		self.focus_on_artifact_trigger.emit(msg)


	def focusOnArtifactMonitor(self, msg):
		'''
		Draw something on the gui in this function
		'''
		#check that threading is working properly
		if (not isinstance(threading.current_thread(), threading._MainThread)):
			print "Drawing on the message panel not guarented to be on the proper thread"			

			
	def shutdown_plugin(self):
		# TODO unregister all publishers here
		pass
		
