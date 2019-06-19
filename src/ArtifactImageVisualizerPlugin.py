#!/usr/bin/env python

"""
Plugin to visualize artifact images
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebastion or Matt).
"""

import rospy
import rospkg
import threading
import cv2

from cv_bridge import CvBridge

import python_qt_binding.QtWidgets as qt
import python_qt_binding.QtGui as gui

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QPixmap
from python_qt_binding.QtWidgets import QWidget
from qt_gui.plugin import Plugin
from PyQt5.QtCore import pyqtSignal

from std_msgs.msg import String, UInt8, Bool
from basestation_gui_python.msg import (
    GuiMessage,
    DarpaStatus,
    ArtifactDisplayImage,
    RetreiveArtifactImage,
    ArtifactVisualizerUpdate,
)


class ArtifactImageVisualizerPlugin(Plugin):

    disp_image_trigger = pyqtSignal(object)  # to keep the drawing on the proper thread
    update_art_label_trigger = pyqtSignal(object)

    def __init__(self, context):
        super(ArtifactImageVisualizerPlugin, self).__init__(context)
        self.setObjectName("ArtifactImageVisualizerPlugin")

        # the image size to display image artifacts
        self.artifact_img_width, self.artifact_img_length = [640, 360]

        self.initPanel(context)  # layout plugin

        self.br = CvBridge()  # bridge from opencv to ros image messages

        self.disp_image_trigger.connect(self.dispImageMonitor)
        self.update_art_label_trigger.connect(self.updateArtLabelMonitor)

        # setup subscribers/pubs
        self.img_request_pub = rospy.Publisher(
            "/gui/change_disp_img", RetreiveArtifactImage, queue_size=10
        )

        self.image_sub = rospy.Subscriber(
            "/gui/img_to_display", ArtifactDisplayImage, self.dispImage
        )
        self.update_panel_sub = rospy.Subscriber(
            "/gui/update_art_label", ArtifactVisualizerUpdate, self.updateArtLabel
        )
        self.clear_panel_sub = rospy.Subscriber(
            "/gui/clear_img", Bool, self.dispBlankImg
        )  # make the image blank again

    def initPanel(self, context):
        """Initialize the panel for displaying widgets."""
        self.artvis_widget = QWidget()
        self.artvis_layout = qt.QGridLayout()

        self.artvis_widget.setWindowTitle("Artifact Visualization")
        context.add_widget(self.artvis_widget)

        # add in a blank label to represent an object
        self.art_image = qt.QLabel()

        rospack = rospkg.RosPack()
        img_filename = rospack.get_path("basestation_gui_python") + "/src/black_img.png"
        width_length = (self.artifact_img_width, self.artifact_img_length)

        self.blank_img = cv2.imread(img_filename)
        self.blank_img = cv2.resize(self.blank_img, width_length)

        img_height, img_width = self.blank_img.shape[:2]
        img = gui.QImage(
            self.blank_img, img_width, img_height, gui.QImage.Format_RGB888
        )
        img = gui.QPixmap.fromImage(img)
        self.art_image.setPixmap(img)
        self.art_image.mousePressEvent = self.publishImageCoord

        # last 2 parameters are rowspan and columnspan
        self.artvis_layout.addWidget(self.art_image, 1, 0, 3, 3)

        self.update_art_label = qt.QLabel()
        self.update_art_label.setText("Updates here")
        self.update_art_label.setAlignment(Qt.AlignCenter)
        self.update_art_label.setStyleSheet("background-color: rgba(126, 126, 126,50%)")
        self.update_art_label.hide()
        self.artvis_layout.addWidget(self.update_art_label, 3, 0, 1, 3)

        boldFont = gui.QFont()
        boldFont.setBold(True)

        # add arrow buttons and label in the middle to indicate what image we're on
        self.img_back_button = qt.QPushButton("<-")
        self.img_back_button.clicked.connect(self.imgBack)
        self.artvis_layout.addWidget(self.img_back_button, 4, 0)

        self.img_displayed_label = qt.QLabel()
        self.img_displayed_label.setText("Img 0/0")
        self.img_displayed_label.setFont(gui.QFont(None, 16, gui.QFont.Bold))
        self.img_displayed_label.setAlignment(Qt.AlignCenter)
        self.artvis_layout.addWidget(self.img_displayed_label, 4, 1)

        self.img_forward_button = qt.QPushButton("->")
        self.img_forward_button.clicked.connect(self.imgForward)
        self.artvis_layout.addWidget(self.img_forward_button, 4, 2)

        # add to the overall gui
        self.artvis_widget.setLayout(self.artvis_layout)

    ############################################################################################
    # Functions for iterating over the images
    ############################################################################################

    def imgBack(self):
        """Get the previous image in the sequence."""
        msg = RetreiveArtifactImage()
        msg.data = RetreiveArtifactImage.DIRECTION_BACKWARD
        self.img_request_pub.publish(msg)

    def imgForward(self):
        """Get the next image in the sequence."""
        msg = RetreiveArtifactImage()
        msg.data = RetreiveArtifactImage.DIRECTION_FORWARD
        self.img_request_pub.publish(msg)

    ############################################################################################
    # Threaded functions for changing the image or update panel
    ############################################################################################

    def dispBlankImg(self, msg):
        """
        Used to display a blank black image. Useful when we submit an artifact and 
        this panel should be cleared to designate that we are no longer viewing that artifact

        msg is a Bool which should always be true. Basically to just call this callback.
        """
        if msg.data == True:  # basically anything can be sent. This is just a default
            # make the image blank
            msg = ArtifactDisplayImage()
            msg.img = self.br.cv2_to_imgmsg(self.blank_img)
            self.dispImage(msg)

    def dispImage(self, msg):
        """For threading. See function below."""
        self.disp_image_trigger.emit(msg)

    def dispImageMonitor(self, msg):
        """
        Function to display image of an artifact

        msg is a custom ArtifactDisplayImage which contains an image, and the number in the 
        artifact's image list
        """

        # check that threading is working properly
        if not isinstance(threading.current_thread(), threading._MainThread):
            rospy.logerr("[Artifact Visualization] Not rendering on main thread.")

        img = self.br.imgmsg_to_cv2(msg.img)

        if len(img.shape) > 2:  # if there's actually an image
            img = cv2.resize(img, (self.artifact_img_width, self.artifact_img_length))

            img_height, img_width = img.shape[:2]
            img = gui.QImage(img, img_width, img_height, gui.QImage.Format_RGB888)
            img = gui.QPixmap.fromImage(img)
            self.art_image.setPixmap(img)

        # change the label
        if msg.num_images == 0:
            image_ind = 0
        else:
            image_ind = msg.image_ind + 1

        self.img_displayed_label.setText(
            "Img " + str(image_ind) + "/" + str(msg.num_images)
        )

    def updateArtLabel(self, msg):
        """For proper threading. See function below."""
        self.update_art_label_trigger.emit(msg)

    def updateArtLabelMonitor(self, msg):
        """
        The art label is a label that shows over the image if the underlying artifact has changed or has
        been deleted

        msg is a string that just contains the text to display
        """

        if msg.data == ArtifactVisualizerUpdate.DELETE:  # the artifact has been deleted
            self.update_art_label.setText("Artifact has been deleted!")
            self.update_art_label.setStyleSheet(
                "background-color: rgba(220, 0, 0, 70%)"
            )
            self.update_art_label.show()

        # the artifact has been modified
        elif msg.data == ArtifactVisualizerUpdate.UPDATE:
            self.update_art_label.setText(
                "Artifact has been updated. Please select it again in the queue"
            )
            self.update_art_label.setStyleSheet(
                "background-color: rgba(255, 255, 0, 80%)"
            )
            self.update_art_label.show()

        # the user has done something else, remove the label from sight
        elif msg.data == ArtifactVisualizerUpdate.HIDE:
            self.update_art_label.hide()

    ############################################################################################
    # Misc. functions
    ############################################################################################

    def publishImageCoord(self, event):
        """
        At some point there was a request for image coordinate clicks to be published,
        in order to possibly extract 3d info.  The backend was not written for this, so
        this function currently does nothing.
        """
        pass

    def shutdown_plugin(self):
        self.clear_panel_sub.unregister()
        self.update_panel_sub.unregister()
        self.image_sub.unregister()
