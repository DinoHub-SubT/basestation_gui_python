#!/usr/bin/env python

"""
RQT Plugin for inspecting artifacts (looking at images, positions, etc.)
and changing artifact properties (position, category, etc.)
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebastion or Matt).
"""
from __future__ import print_function

import rospy
import rospkg
from std_msgs.msg import String, Bool
import threading

from qt_gui.plugin import Plugin
import python_qt_binding.QtWidgets as qt
import python_qt_binding.QtCore as core
import python_qt_binding.QtGui as gui

from python_qt_binding.QtCore import Slot, Qt, qVersion, qWarning, Signal
from python_qt_binding.QtGui import QColor, QPixmap
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy

from functools import partial
import pdb
import yaml
from PyQt5.QtCore import pyqtSignal

from basestation_gui_python.msg import GuiMessage, DarpaStatus, Artifact, ArtifactUpdate


class ArtifactManipulationPlugin(Plugin):

    focus_on_artifact_trigger = pyqtSignal(
        object
    )  # to keep the drawing on the proper thread

    def __init__(self, context):
        super(ArtifactManipulationPlugin, self).__init__(context)
        self.setObjectName("ArtifactManipulationPlugin")

        # get the artifact categories and priorities
        rospack = rospkg.RosPack()
        config_filename = (
            rospack.get_path("basestation_gui_python") + "/config/gui_params.yaml"
        )
        config = yaml.load(open(config_filename, "r").read())

        self.artifact_categories, self.artifact_priorities = [], []

        exp_params = config["darpa_params"]

        for name in exp_params["artifact_categories"]:
            self.artifact_categories.append(name)

        gui_params = config["experiment_params"]

        for name in gui_params["artifact_priorities"]:
            self.artifact_priorities.append(name)

        self.artifact_id_displayed = None

        # if we're also simulating the darpa command post
        self.connect_to_command_post = rospy.get_param("/connect_to_command_post")

        self.initPanel(context)  # layout plugin

        self.focus_on_artifact_trigger.connect(self.focusOnArtifactMonitor)

        # setup subscribers/publishers
        self.update_artifact_info_pub = rospy.Publisher(
            "/gui/update_artifact_info", ArtifactUpdate, queue_size=10
        )  # for if we get more info from robot
        self.message_pub = rospy.Publisher(
            "/gui/message_print", GuiMessage, queue_size=10
        )  # for if we need to print a message
        self.submit_pub = rospy.Publisher(
            "/gui/submit_artifact_from_manip_plugin", String, queue_size=10
        )  # to tell other plugins we submitted an artifact

        self.refresh_sub = rospy.Subscriber(
            "/gui/refresh_manipulation_info", Artifact, self.focusOnArtifact
        )  # the information relevant to manipulation for the artifact in focus has been changed
        self.disable_sequence_sub = rospy.Subscriber(
            "/gui/disable_confirm_cancel_manip_plugin", Bool, self.cancelProposal
        )  # disable sequence if something happened in another plugin

    def initPanel(self, context):
        """
		Initialize the panel for displaying widgets
		"""

        # define the overall widget
        self.artmanip_widget = QWidget()
        self.artmanip_layout = qt.QGridLayout()

        context.add_widget(self.artmanip_widget)

        boldFont = gui.QFont()
        boldFont.setBold(True)

        # add in information about 3d position
        robot_pos = ["N/A", "N/A", "N/A"]  # fake data

        self.orig_pos_label = qt.QLabel()
        self.orig_pos_label.setText("\n\nOriginal Position (XYZ)")
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

        # editable information about the position, to send to darpa
        self.refined_pos_label = qt.QLabel()
        self.refined_pos_label.setText("\nRefined Position (XYZ)")
        self.refined_pos_label.setFont(boldFont)
        self.refined_pos_label.setAlignment(Qt.AlignCenter)
        self.artmanip_layout.addWidget(self.refined_pos_label, 3, 0, 1, 3)

        self.art_pos_textbox_x, self.art_pos_textbox_y, self.art_pos_textbox_z = (
            qt.QLineEdit(),
            qt.QLineEdit(),
            qt.QLineEdit(),
        )

        # any change in any pose box will update the artifact's entire psoe with
        # the values from the other pose textboxes as well

        # editingFinished waits until all of the text has been typed into the box, so this
        # doesn't fire at every character input into the textboxes
        self.art_pos_textbox_x.editingFinished.connect(
            partial(self.updateArtifactPose, 0)
        )
        self.art_pos_textbox_y.editingFinished.connect(
            partial(self.updateArtifactPose, 1)
        )
        self.art_pos_textbox_z.editingFinished.connect(
            partial(self.updateArtifactPose, 2)
        )

        # fill in some fake data
        self.art_pos_textbox_x.setText(str(robot_pos[0]))
        self.artmanip_layout.addWidget(self.art_pos_textbox_x, 4, 0)

        self.art_pos_textbox_y.setText(str(robot_pos[1]))
        self.artmanip_layout.addWidget(self.art_pos_textbox_y, 4, 1)

        self.art_pos_textbox_z.setText(str(robot_pos[2]))
        self.artmanip_layout.addWidget(self.art_pos_textbox_z, 4, 2)

        # add button for displaying the interactive marker
        self.art_refinement_button = qt.QPushButton("Show Refinement Marker")
        self.art_refinement_button.setCheckable(True)
        self.art_refinement_button.clicked.connect(self.processArtRefinementPress)
        self.artmanip_layout.addWidget(self.art_refinement_button, 5, 0, 1, 3)

        # add in a few buttons at the bottom to change category/priority
        self.artmanip_button_layout = qt.QHBoxLayout()
        self.artmanip_button_list = []

        artifact_category_label = qt.QLabel()
        artifact_category_label.setText("\n\nCategory")
        artifact_category_label.setFont(boldFont)
        artifact_category_label.setAlignment(Qt.AlignCenter)
        self.artmanip_layout.addWidget(artifact_category_label, 6, 0, 1, 3)

        # artifact_priority_label = qt.QLabel()
        # artifact_priority_label.setText("\n\nPriority")
        # artifact_priority_label.setFont(boldFont)
        # artifact_priority_label.setAlignment(Qt.AlignCenter)
        # self.artmanip_layout.addWidget(artifact_priority_label, 6, 2)

        # make the combobox for setting the artifact category
        self.artifact_cat_box = qt.QComboBox()

        for category in self.artifact_categories:
            self.artifact_cat_box.addItem(category)

        self.artifact_cat_box.currentTextChanged.connect(self.updateArtifactCat)

        self.artmanip_layout.addWidget(self.artifact_cat_box, 7, 0, 1, 3)

        # make the combobox for setting the artifact priority
        # self.artifact_priority_box = qt.QComboBox()

        # self.artifact_priority_box.addItem("High")
        # self.artifact_priority_box.addItem("Med")
        # self.artifact_priority_box.addItem("Low")

        # self.artifact_priority_box.currentTextChanged.connect(
        #     self.updateArtifactPriority
        # )
        # self.artmanip_layout.addWidget(self.artifact_priority_box, 7, 2)

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
        self.darpa_cancel_button.clicked.connect(
            partial(self.cancelProposal, Bool(True))
        )
        self.darpa_cancel_button.setStyleSheet("background-color:rgb(100, 100, 100)")
        self.darpa_cancel_button.setEnabled(False)
        self.artmanip_layout.addWidget(self.darpa_cancel_button, 9, 0)

        # add to the overall gui
        self.artmanip_widget.setLayout(self.artmanip_layout)

    ############################################################################################
    # Functions for updating artifact info based on gui interaction
    ############################################################################################

    def updateArtifactPose(self, element):
        """
		Update the artifact's pose after the line edit box contents have been changed

		element defines which element i the pose we update
			0 is x
			1 is y
			2 is z
		"""

        if self.artifact_id_displayed != None:  # we're actually displaying an artifact

            msg = ArtifactUpdate()

            msg.unique_id = self.artifact_id_displayed

            if element == 0:
                msg.update_type = ArtifactUpdate.PROPERTY_POSE_X
                msg.curr_pose.position.x = float(self.art_pos_textbox_x.text())

            elif element == 1:
                msg.update_type = ArtifactUpdate.PROPERTY_POSE_Y
                msg.curr_pose.position.y = float(self.art_pos_textbox_y.text())

            elif element == 2:
                msg.update_type = ArtifactUpdate.PROPERTY_POSE_Z
                msg.curr_pose.position.z = float(self.art_pos_textbox_z.text())

            else:
                update_msg = GuiMessage()
                update_msg.data = "We received an update to artifact pose with unknown indice (should be 0, 1, or 2). Artifact not updated"
                update_msg.color = update_msg.COLOR_RED
                self.message_pub.publish(update_msg)

                return

            self.update_artifact_info_pub.publish(msg)

    def updateArtifactCat(self):
        """
		Change the artifact category after the combox box has been changed
		"""

        if self.artifact_id_displayed != None:  # we're actually displaying an artifact

            msg = ArtifactUpdate()

            msg.unique_id = self.artifact_id_displayed
            msg.update_type = ArtifactUpdate.PROPERTY_CATEGORY
            msg.category = self.artifact_cat_box.currentText()

            self.update_artifact_info_pub.publish(msg)

    def updateArtifactPriority(self):
        """
		Change the artifact priority after the combo box has been changed
		"""
        if self.artifact_id_displayed != None:  # we're actually displaying an artifact

            msg = ArtifactUpdate()

            msg.unique_id = self.artifact_id_displayed
            msg.update_type = ArtifactUpdate.PROPERTY_PRIORITY
            # msg.priority = self.artifact_priority_box.currentText()

            self.update_artifact_info_pub.publish(msg)

    ############################################################################################
    # Functions for submitting an artifact
    ############################################################################################

    def decideArtifact(self):
        """
		If we're displaying an artifact, unlock the Confirm/Cancel buttons which will
		eventually allow us to submit an artifact to DARPA
		"""
        if self.artifact_id_displayed == None:
            update_msg = GuiMessage()
            update_msg.data = (
                "Nothing proposed. Please select and artifact from the queue"
            )
            update_msg.color = update_msg.COLOR_ORANGE
            self.message_pub.publish(update_msg)

        elif self.connect_to_command_post:

            # enable the confirm and cancel buttons
            self.darpa_confirm_button.setEnabled(True)
            self.darpa_cancel_button.setEnabled(True)

            # color the buttons appropriately
            self.darpa_confirm_button.setStyleSheet("background-color:rgb(0,220,0)")
            self.darpa_cancel_button.setStyleSheet("background-color:rgb(220,0,0)")

        else:
            update_msg = GuiMessage()
            update_msg.data = (
                "Not connected to darpa basestation. Thus artifatc not submitted."
            )
            update_msg.color = update_msg.COLOR_ORANGE
            self.message_pub.publish(update_msg)

    def proposeArtifact(self):
        """
		Propose an artifact to DARPA 
		"""

        if self.artifact_id_displayed != None:

            msg = String()
            msg.data = self.artifact_id_displayed

            self.submit_pub.publish(msg)

            # reset the darpa proposal buttons
            self.darpa_confirm_button.setEnabled(False)
            self.darpa_cancel_button.setEnabled(False)

            self.darpa_confirm_button.setStyleSheet(
                "background-color:rgb(126, 126, 126)"
            )
            self.darpa_cancel_button.setStyleSheet(
                "background-color:rgb(126, 126, 126)"
            )

            # clear the various boxes
            self.orig_pos_label_x.setText("")
            self.orig_pos_label_y.setText("")
            self.orig_pos_label_z.setText("")

            self.art_pos_textbox_x.setText("")
            self.art_pos_textbox_y.setText("")
            self.art_pos_textbox_z.setText("")

            # reset the focused artifact id
            self.artifact_id_displayed = None

    def cancelProposal(self, msg):
        """
		Cancel an artifact proposal, which simply amounts to disabling 
		the confirm/cancel buttons. Can be called from a button press
		or when we select another artifact from the queue, these buttons
		should de-activate

		msg is a Bool, should always be true
		"""

        if msg.data == True:
            # reset the darpa proposal buttons
            self.darpa_confirm_button.setEnabled(False)
            self.darpa_cancel_button.setEnabled(False)

            self.darpa_confirm_button.setStyleSheet(
                "background-color:rgb(126, 126, 126)"
            )
            self.darpa_cancel_button.setStyleSheet(
                "background-color:rgb(126, 126, 126)"
            )

    ############################################################################################
    # Functions for changing various widget's values based ont he artifact we just selected
    # from the queue
    ############################################################################################

    def focusOnArtifact(self, msg):
        """
		For proper threading. See function below.
		"""
        self.focus_on_artifact_trigger.emit(msg)

    def focusOnArtifactMonitor(self, msg):
        """
		Update the information displayed for manipulating re artifact (position, category, etc.)

		msg is custom msg type Artifact
		"""
        # check that threading is working properly
        if not isinstance(threading.current_thread(), threading._MainThread):
            print(
                "Drawing on the message panel not guarented to be on the proper thread"
            )

        self.artifact_id_displayed = msg.unique_id

        # change the original and refined positions

        self.orig_pos_label_x.setText(str(round(msg.orig_pose.position.x, 1)))
        self.orig_pos_label_y.setText(str(round(msg.orig_pose.position.y, 1)))
        self.orig_pos_label_z.setText(str(round(msg.orig_pose.position.z, 1)))

        self.art_pos_textbox_x.setText(str(round(msg.curr_pose.position.x, 1)))
        self.art_pos_textbox_y.setText(str(round(msg.curr_pose.position.y, 1)))
        self.art_pos_textbox_z.setText(str(round(msg.curr_pose.position.z, 1)))

        # change the priority
        # priority_ind = self.artifact_priority_box.findText(
        #     msg.priority, core.Qt.MatchFixedString
        # )

        # if priority_ind >= 0:
        #     self.artifact_priority_box.setCurrentIndex(priority_ind)

        # change the category
        category_ind = self.artifact_cat_box.findText(
            msg.category, core.Qt.MatchFixedString
        )

        if category_ind >= 0:
            self.artifact_cat_box.setCurrentIndex(category_ind)

        # publish that stuff has changed, so the handler can keep track of stuff
        # the normal publisher doesn't necessarily work because the artifact_id_displayed
        # may not necessarily have been pressed
        art_update_msg = ArtifactUpdate()
        art_update_msg.unique_id = self.artifact_id_displayed

        art_update_msg.update_type = ArtifactUpdate.PROPERTY_POSE_X
        art_update_msg.curr_pose.position.x = float(self.art_pos_textbox_x.text())
        self.update_artifact_info_pub.publish(art_update_msg)

        art_update_msg.update_type = ArtifactUpdate.PROPERTY_POSE_Y
        art_update_msg.curr_pose.position.y = float(self.art_pos_textbox_y.text())
        self.update_artifact_info_pub.publish(art_update_msg)

        art_update_msg.update_type = ArtifactUpdate.PROPERTY_POSE_Z
        art_update_msg.curr_pose.position.z = float(self.art_pos_textbox_z.text())
        self.update_artifact_info_pub.publish(art_update_msg)

        art_update_msg.update_type = ArtifactUpdate.PROPERTY_CATEGORY
        art_update_msg.category = self.artifact_cat_box.currentText()
        self.update_artifact_info_pub.publish(art_update_msg)

    ############################################################################################
    # Misc. functions
    ############################################################################################

    def processArtRefinementPress(self):
        pass

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.refresh_sub.unregister()
        self.disable_sequence_sub.unregister()
