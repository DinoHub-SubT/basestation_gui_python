#!/usr/bin/env python

"""
The info panel in the gui (which displays time remaining, score, remaining reports)
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebation or Matt).
"""
import rospy
import threading

import python_qt_binding.QtWidgets as qt
import python_qt_binding.QtCore as core
import python_qt_binding.QtGui as gui

from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import Qt
from PyQt5.QtCore import pyqtSignal
from gui_utils import displaySeconds

from std_msgs.msg import String
from basestation_gui_python.msg import GuiMessage, DarpaStatus


class InfoPlugin(Plugin):
    # to keep the message printing on the proper thread
    info_trigger = pyqtSignal(object)

    def __init__(self, context):
        super(InfoPlugin, self).__init__(context)
        self.setObjectName("InfoPlugin")

        self.initPanel(context)
        self.info_trigger.connect(self.setInfoStringMonitor)
        self.info_string = None
        self.info_sub = rospy.Subscriber(
            "/gui/darpa_status", DarpaStatus, self.setInfoString
        )  # for gathering info from darpa

    def initPanel(self, context):
        """Initialize the panel for displaying widgets."""
        self.info_box_widget = qt.QWidget()
        self.info_box_layout = qt.QGridLayout()

        self.info_box_widget.setWindowTitle("Scoring")
        context.add_widget(self.info_box_widget)

        bold_font = gui.QFont()
        bold_font.setBold(True)
        bold_font.setPointSize(16)

        self.info_label = qt.QLabel()
        self.info_label.setText("Time: -- \t Score: -- \t Remaining: --")
        self.info_label.setAlignment(Qt.AlignCenter)
        self.info_label.setFont(bold_font)
        self.info_label.setStyleSheet("border:3px solid rgb(0, 0, 0);")
        self.info_box_layout.addWidget(self.info_label, 1, 0)
        self.info_box_widget.setLayout(self.info_box_layout)

    def setInfoString(self, msg):
        """For proper threading. See function below."""
        self.info_trigger.emit(msg)

    def setInfoStringMonitor(self, msg):
        """
        Update the information string displaying darpa information.

        msg is a custom DarpaStatus msg containing info about the time_elpased, score
        and remaining reports as received from DARPA
        """
        if not isinstance(threading.current_thread(), threading._MainThread):
            rospy.logerr("[Scoring] Not rendering on main thread.")

        sec = str(displaySeconds(msg.time_elapsed))
        score = str(msg.score)
        rem = str(msg.remaining_reports)
        text = "Time: {0}\t Score: {1}\t Remaining: {2}".format(sec, score, rem)
        self.info_label.setText(text)

    def shutdown_plugin(self):
        self.info_sub.unregister()
