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


class RobotStatusPlugin(Plugin):
    # to keep the drawing on the proper thread
    status_update_trigger = pyqtSignal(object, object)

    def __init__(self, context):
        super(RobotStatusPlugin, self).__init__(context)
        self.setObjectName("RobotStatusPlugin")

        config = robots.Config()
        names = [robot.name for robot in config.robots]

        (widget, table) = self.initPanel(names)
        context.add_widget(widget)

        self.subscriptions = []
        self.status_table = table
        self.status_update_trigger.connect(self.statusUpdateMonitor)

        def sub(where, what, callback):
            s = rospy.Subscriber(where, what, callback)
            self.subscriptions.append(s)

        def update(column):
            def fn(msg):
                self.status_update_trigger.emit(column, msg)

            return fn

        for (column, r) in enumerate(config.robots):
            topic = "/{0}/{1}".format(r.topic_prefix, r.topics.get("status_update"))
            sub(topic, bsm.StatusUpdate, update(column))

    def initPanel(self, robot_names):
        """Initialize the panel for displaying widgets."""
        widget = QWidget()
        layout = qt.QGridLayout()
        status = qt.QLabel()
        table = qt.QTableWidget()
        statuses = [
            "Battery(mins)",
            "Comms",
            "Mobility",
            "CPU",
            "Disk Space",
            "RSSI",
        ]

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
