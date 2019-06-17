#!/usr/bin/env python

"""
Plugin to display status information from the robot
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebation or Matt).
"""
import rospy
import threading
import robots

import python_qt_binding.QtWidgets as qt
import python_qt_binding.QtGui as gui

from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QWidget
from PyQt5.QtCore import pyqtSignal

import basestation_msgs.msg as bsm

from basestation_gui_python.msg import StatusPanelUpdate


class RobotStatusPlugin(Plugin):
    # to keep the drawing on the proper thread
    robot_status_trigger = pyqtSignal(object)
    status_update_trigger = pyqtSignal(object, object)

    def __init__(self, context):
        super(RobotStatusPlugin, self).__init__(context)
        self.setObjectName("RobotStatusPlugin")
        self.config = robots.Config()

        names = [robot.name for robot in self.config.robots]

        # Define the rows of the table as statuses that the robot will have.
        self.statuses = [
            "Battery(mins)",
            "Comms",
            "Mobility",
            "CPU",
            "Disk Space",
            "RSSI",
        ]

        (widget, table) = self.initPanel(names, self.statuses)
        context.add_widget(widget)

        self.status_table = table
        self.robot_status_trigger.connect(self.robotStatusMonitor)
        self.status_update_trigger.connect(self.statusUpdateMonitor)

        self.subscriptions = []

        def sub(where, what, callback):
            s = rospy.Subscriber(where, what, callback)
            self.subscriptions.append(s)

        def update(column):
            def fn(msg):
                self.status_update_trigger.emit(column, msg)

            return fn

        sub("/status_panel_update", StatusPanelUpdate, self.robotStatus)
        for (column, r) in enumerate(self.config.robots):
            topic = "/{0}/{1}".format(r.topic_prefix, r.topics.get("status_update"))
            sub(topic, bsm.StatusUpdate, update(column))

    def initPanel(self, robot_names, statuses):
        """Initialize the panel for displaying widgets."""
        widget = QWidget()
        layout = qt.QGridLayout()
        status = qt.QLabel()
        table = qt.QTableWidget()

        widget.setLayout(layout)
        layout.addWidget(status, 0, 0)
        layout.addWidget(table, 1, 0)

        status.setText("STATUS PANEL")
        status.setAlignment(Qt.AlignCenter)

        table.horizontalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)
        table.verticalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)
        table.setRowCount(len(statuses))
        table.setColumnCount(len(robot_names))
        table.setVerticalHeaderLabels(statuses)
        table.setHorizontalHeaderLabels(robot_names)

        return (widget, table)

    def robotStatus(self, msg):
        """For proper threading. See function below."""
        self.robot_status_trigger.emit(msg)

    def robotStatusMonitor(self, msg):
        """
	Display some status information about the robot. Update a cell in the status table.

	Msg is a custom StatusPanelUpdate msg containing the robot_number, status field, 
	and status value and color.
        """
        if not isinstance(threading.current_thread(), threading._MainThread):
            rospy.logerr("[Robot Status Plugin] Not rendering on main thread.")

        # The columns of the status table are ordered by the robots list from the
        # config.  Since a robot's exec_id can be changed arbitrarily we need to
        # the appropriate column within the table.
        column = -1
        for (i, r) in enumerate(self.config.robots):
            if r.executive_id == msg.robot_id:
                column = i
                break
        if column == -1:
            rospy.logerr("[Robot Status Plugin] Invalid robot ID %d", msg.robot_id)
            return
        if msg.key not in self.statuses:
            rospy.logerr("[Robot Status Plugin] Invalid robot status: %s", msg.key)
            return
        row = self.statuses.index(msg.key)
        rgb = gui.QColor(rgb.r, rgb.g, rgb.b)
        self.updatePanel(row, column, msg.value, rgb)

    def statusUpdateMonitor(self, column, msg):
        rgb = gui.QColor(0, 255, 0)
        if msg.severity == bsm.StatusUpdate.SEVERITY_WARNING:
            rgb = gui.QColor(255, 255, 0)
        elif msg.severity == bsm.StatusUpdate.SEVERITY_CRITICAL:
            rgb = gui.QColor(255, 0, 0)
        self.updatePanel(msg.what, column, msg.value, rgb)

    def updatePanel(self, row, column, value, color):
        item = self.status_table.item(row, column)
        if item == None:
            self.status_table.setItem(row, column, qt.QTableWidgetItem(""))
        item = self.status_table.item(row, column)
        item.setBackground(color)
        item.setText(value)
        self.status_table.viewport().update()

    def shutdown_plugin(self):
        for s in self.subscriptions:
            s.unregister()
