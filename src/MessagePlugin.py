#!/usr/bin/env python

"""
Plugin to display text messages about various going-ons
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
from PyQt5.QtCore import pyqtSignal
from gui_utils import displaySeconds

from std_msgs.msg import String
from basestation_gui_python.msg import GuiMessage, DarpaStatus


class MessagePlugin(Plugin):
    # to keep the message printing on the proper thread
    print_message_trigger = pyqtSignal(object)

    def __init__(self, context):
        super(MessagePlugin, self).__init__(context)
        self.setObjectName("MessagePlugin")

        self.darpa_time = None

        self.initMessagePanel(context)  # layout plugin

        self.print_message_trigger.connect(
            self.printMessageMonitor
        )  # to print in a thread-safe manner

        # setup subscribers
        self.message_sub = rospy.Subscriber(
            "/gui/message_print", GuiMessage, self.printMessage
        )  # contains message data to disp
        self.time_sub = rospy.Subscriber(
            "/gui/darpa_status", DarpaStatus, self.setDarpaTime
        )  # contains darpa status information

    def initMessagePanel(self, context):
        """Initialize the panel which displays messages."""
        self.message_box_widget = qt.QWidget()
        self.message_box_layout = qt.QGridLayout()

        self.message_box_widget.setWindowTitle("Messages")
        context.add_widget(self.message_box_widget)

        self.message_textbox = qt.QListWidget()
        self.message_textbox.setWordWrap(True)

        self.message_box_layout.addWidget(self.message_textbox, 0, 0)
        self.message_box_widget.setLayout(self.message_box_layout)

    def setDarpaTime(self, msg):
        """
        Function that saves the darpa time published from the darpa status node
        into a local variable to be used here

        msg is a custom DarpaStatus message which contains info about time/score/remaining reports
        """
        self.darpa_time = msg.time_elapsed

    def printMessage(self, msg):
        """For being thread-safe. See function below."""
        self.print_message_trigger.emit(msg)

    def printMessageMonitor(self, msg):
        """
        Add message to the message box that is simply a string from
        this application (not ROS)

        msg is a custom GuiMessage msg which contains a field for text and 
                constants for setting message color, depnding on importance of message
        """
        # check that threading is working properly
        if not isinstance(threading.current_thread(), threading._MainThread):
            rospy.logerr("[Messages] Not rendering on main thread.")

        if self.darpa_time != None:
            item = qt.QListWidgetItem(
                "[" + displaySeconds(self.darpa_time) + "]" + msg.data
            )
        else:  # if we're not connect to the command post, don't display a time
            item = qt.QListWidgetItem("[--] " + msg.data)

        if msg.color == msg.COLOR_ORANGE:
            msg_color = [242, 143, 50]
        elif msg.color == msg.COLOR_RED:
            msg_color = [250, 128, 114]
        elif msg.color == msg.COLOR_GREEN:
            msg_color = [144, 238, 144]
        elif msg.color == msg.COLOR_GRAY:
            msg_color = [220, 220, 220]
        else:
            msg_color = [255, 255, 255]

        item.setBackground(gui.QColor(msg_color[0], msg_color[1], msg_color[2]))

        self.message_textbox.addItem(item)
        self.message_textbox.sortItems(core.Qt.DescendingOrder)
        self.message_textbox.viewport().update()  # refresh the message box

    def shutdown_plugin(self):
        self.message_sub.unregister()
        self.time_sub.unregister()
