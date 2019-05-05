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

class MessagePlugin(Plugin):

	print_message_trigger = pyqtSignal(object, object) #to keep the message printing on the proper thread

	def __init__(self, context):
		super(MessagePlugin, self).__init__(context)
		self.setObjectName('MessagePlugin')

		self.widget = QWidget()
		self.global_widget = qt.QGridLayout()
     
		
		self.widget.setLayout(self.global_widget)
		context.add_widget(self.widget)

		self.initMessagePanel()

		self.print_message_trigger.connect(self.printMessageMonitor)

	def initMessagePanel(self):
		'''
		Initialize the panel which displays various text messages
		'''

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


	def printMessage(self, msg, rgb):
		self.print_message_trigger.emit(msg, rgb)


	def printMessageMonitor(self, msg, rgb=None):
		'''
		Add message to the message box that is simply a string from
		this application (not ROS)
		'''

		print "print message:",isinstance(threading.current_thread(), threading._MainThread)
		print "print message:",threading.current_thread().__class__.__name__
		
		# with self.update_message_box_lock:
		#     self.message_textbox.setSortingEnabled(False)


		if (self.darpa_gui_bridge != None and self.darpa_gui_bridge.darpa_status_update['run_clock'] != None):
			item = qt.QListWidgetItem('['+str(self.darpa_gui_bridge.displaySeconds(self.darpa_gui_bridge.darpa_status_update['run_clock']))+'] '+msg)
		else:
			item = qt.QListWidgetItem('[--] '+msg)

		if (rgb != None):
			item.setBackground(gui.QColor(rgb[0], rgb[1], rgb[2]))

		self.message_textbox.addItem(item)



			# self.message_textbox.setSortingEnabled(True)

			# self.message_textbox.sortItems(core.Qt.DescendingOrder)

		self.message_textbox.viewport().update()


	def select_config_file(self):
		starting_path = os.path.join(rospkg.RosPack().get_path('basestation_gui_python'), 'config')
		filename = qt.QFileDialog.getOpenFileName(self.widget, 'Open Config File', starting_path, "Config Files (*.yaml)")[0]
		if filename != '':
			self.initiateSettings(filename)
			
	def shutdown_plugin(self):
		pass
		# TODO unregister all publishers here
		# if(self.connect_to_command_post):
		#     self.darpa_gui_bridge.shutdownHttpServer()
		

	def save_settings(self, plugin_settings, instance_settings):
		# instance_settings.set_value('config_filename', self.config_filename)
		pass

	def restore_settings(self, plugin_settings, instance_settings):
		'''
		Function which lays out all of the widgets from a pre-specified .yaml 
		config file. 
		'''
		self.initiateSettings(instance_settings.value('config_filename'))
		

	def initiateSettings(self, config_filename):
		'''
		Generates the gui using either start fresh or from using previous settings
		'''


		# self.config_filename = config_filename
		# self.ros_gui_bridge = RosGuiBridge(self.config_filename, self)

		# self.defineRobotTransformButtons()
		
		# config = yaml.load(open(self.config_filename, 'r').read())
		# darpa_params = config['darpa_params']
		# self.associateCatsWithLabels(darpa_params['artifact_labels'])

		# self.darpa_gui_bridge = DarpaGuiBridge(self.config_filename, self)

		# self.buildGui()        
		
		# self.gui_engine = GuiEngine(self)

		# self.ros_gui_bridge.initSubscribers(self)