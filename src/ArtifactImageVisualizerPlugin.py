#!/usr/bin/python

'''
Plugin to visualize artifact images
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebastion or Matt).
'''

import rospy
import rospkg
from std_msgs.msg import String, UInt8
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

from basestation_gui_python.msg import GuiMessage, DarpaStatus, ArtifactDisplayImage 
import cv2
from cv_bridge import CvBridge

class ArtifactImageVisualizerPlugin(Plugin):

	disp_image_trigger = pyqtSignal(object) #to keep the drawing on the proper thread
	update_art_label_trigger = pyqtSignal(object)

	def __init__(self, context):
		super(ArtifactImageVisualizerPlugin, self).__init__(context)
		self.setObjectName('ArtifactImageVisualizerPlugin')		

		#the image size to display image artifacts
		self.artifact_img_width, self.artifact_img_length = [640, 360]

		self.initPanel(context) #layout plugin

		self.br = CvBridge() #bridge from opencv to ros image messages


		#setup subscribers
		self.image_sub = rospy.Subscriber('/gui/img_to_display', ArtifactDisplayImage, self.dispImage)
		rospy.Subscriber('/gui/update_art_label', String, self.updateArtLabel)

		self.img_request_pub = rospy.Publisher('/gui/change_disp_img', UInt8, queue_size=10)

		self.disp_image_trigger.connect(self.dispImageMonitor)
		self.update_art_label_trigger.connect(self.updateArtLabelMonitor)



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
		self.artvis_widget = QWidget()
		self.artvis_layout = qt.QGridLayout()

		#exapnd to fill the space vertically
		art_label = qt.QLabel()
		art_label.setText('ARTIFACT VISUALIZATION')
		art_label.setAlignment(Qt.AlignCenter)
		self.artvis_layout.addWidget(art_label, 0, 0, 1, 3)

		#add in a blank label to represent an object
		self.art_image = qt.QLabel()
		
		rospack = rospkg.RosPack()
		img_filename = rospack.get_path('basestation_gui_python')+'/src/black_img.png'

		img = cv2.imread(img_filename)
		img = cv2.resize(img,(self.artifact_img_width, self.artifact_img_length))

		img_height, img_width = img.shape[:2]
		img = gui.QImage(img, img_width, img_height, gui.QImage.Format_RGB888)
		img = gui.QPixmap.fromImage(img)
		self.art_image.setPixmap(img)
		self.art_image.mousePressEvent = self.publishImageCoord

		self.artvis_layout.addWidget(self.art_image, 1, 0, 3, 3) #last 2 parameters are rowspan and columnspan

		self.update_art_label = qt.QLabel()
		self.update_art_label.setText('Updates here')
		self.update_art_label.setAlignment(Qt.AlignCenter)
		self.update_art_label.setStyleSheet("background-color: rgba(126, 126, 126,50%)")
		self.update_art_label.hide()
		self.artvis_layout.addWidget(self.update_art_label, 3, 0, 1, 3)

		boldFont = gui.QFont()
		boldFont.setBold(True) 

		#add arrow buttons and label in the middle to indicate what image we're on
		self.img_back_button = qt.QPushButton("<-")
		self.img_back_button.clicked.connect(self.imgBack)
		self.artvis_layout.addWidget(self.img_back_button, 4, 0 )

		self.img_displayed_label = qt.QLabel()
		self.img_displayed_label.setText('Img 0/0')
		self.img_displayed_label.setFont(gui.QFont(None, 16, gui.QFont.Bold) )
		self.img_displayed_label.setAlignment(Qt.AlignCenter)
		self.artvis_layout.addWidget(self.img_displayed_label, 4, 1)

		self.img_forward_button = qt.QPushButton("->")
		self.img_forward_button.clicked.connect(self.imgForward)
		self.artvis_layout.addWidget(self.img_forward_button, 4, 2 )

		#add to the overall gui
		self.artvis_widget.setLayout(self.artvis_layout)
		self.global_widget.addWidget(self.artvis_widget)

	def imgBack(self):
		'''
		Get the previous image in the sequence
		'''
		msg = UInt8()
		msg.data = 1
		self.img_request_pub.publish(msg)

	def imgForward(self):
		'''
		Get the next image in the sequence
		'''
		msg = UInt8()
		msg.data = 0
		self.img_request_pub.publish(msg)	

	def publishImageCoord(self, event):
		'''
		At some point there was a request for image coordinate clicks
		to be published, in order to possibly extract 3d info. The backend was not written
		for this, so this function currently does nothing
		'''

		pass
	
	def dispImage(self, msg):
		self.disp_image_trigger.emit(msg)

	def dispImageMonitor(self, msg):
		'''
		Function to display image of an artifact
		'''
		#check that threading is working properly
		if (not isinstance(threading.current_thread(), threading._MainThread)):
			print "Drawing on the message panel not guarented to be on the proper thread"	

		img = self.br.imgmsg_to_cv2(msg.img)

		if (len(img.shape) > 2): #if there's actually an image
			img = cv2.resize(img,(self.artifact_img_width, self.artifact_img_length))

			img_height, img_width = img.shape[:2]
			img = gui.QImage(img, img_width, img_height, gui.QImage.Format_RGB888)
			img = gui.QPixmap.fromImage(img)
			self.art_image.setPixmap(img)

		#change the label
		if (msg.num_images == 0):
			image_ind = 0
		else:
			image_ind = msg.image_ind + 1

		self.img_displayed_label.setText('Img '+str(image_ind)+'/'+str(msg.num_images))


	def updateArtLabel(self, msg):
		self.update_art_label_trigger.emit(msg)

	def updateArtLabelMonitor(self, msg):
		'''
		The art label is a label that shows over the image if the underlying artifact has changed or has
		been deleted

		msg is a string that just contains the text to display
		'''

		if (msg.data.find('elete') != -1): #the artifact has been deleted
			self.update_art_label.setText('Artifact has been deleted!')
			self.update_art_label.setStyleSheet("background-color: rgba(220, 0, 0, 70%)")
			self.update_art_label.show()

		elif(msg.data.find('updated') != -1): #the artifact has been modified
			self.update_art_label.setText('Artifact has been updated. Please select it again in the queue')
			self.update_art_label.setStyleSheet("background-color: rgba(255, 255, 0, 80%)")
			self.update_art_label.show()


		elif (msg.data =='hide'): #the user has done something else, remove the label from sight
			self.update_art_label.hide()



			
	def shutdown_plugin(self):
		# TODO unregister all publishers here
		pass
		
