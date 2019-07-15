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
import re

import python_qt_binding.QtWidgets as qt
import python_qt_binding.QtGui as gui
import python_qt_binding.QtCore as core

from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QWidget
from PyQt5.QtCore import pyqtSignal

from basestation_msgs.msg import StatusUpdate
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

RED = gui.QColor(255, 0, 0)
YELLOW = gui.QColor(255, 255, 0)
GREEN = gui.QColor(0, 255, 0)


class RobotStatusPlugin(Plugin):
    # to keep the drawing on the proper thread
    status_update_trigger = pyqtSignal(object, object)
    diagnostics_trigger = pyqtSignal(object, object)

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
        self.diagnostics_trigger.connect(self.diagnosticsMonitor)

        def sub(where, what, callback):
            s = rospy.Subscriber(where, what, callback)
            self.subscriptions.append(s)

        def update(column):
            def fn(msg):
                self.status_update_trigger.emit(column, msg)

            return fn

        prefix_columns = dict()

        def diagnostic(msg):
            self.diagnostics_trigger.emit(prefix_columns, msg)

        for (column, r) in enumerate(config.robots):
            status = "/{0}/{1}".format(r.topic_prefix, r.topics.get("status_update"))
            prefix_columns[r.topic_prefix] = column
            sub(status, StatusUpdate, update(column))

        sub("/diagnostics_agg", DiagnosticArray, diagnostic)

    def initPanel(self, robot_names):
        """Initialize the panel for displaying widgets."""
        widget = QWidget()
        layout = qt.QGridLayout()
        table = qt.QTableWidget()
        statuses = ["Battery", "Comms", "Nodes", "CPU", "Disk", "RSSI"]

        widget.setWindowTitle("Robot Status")
        widget.setLayout(layout)
        layout.addWidget(table, 0, 0)

        rows = len(statuses)
        cols = len(robot_names)
        mode = qt.QAbstractItemView.NoSelection

        table.setSelectionMode(mode)
        table.horizontalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)
        table.verticalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)
        table.setRowCount(rows)
        table.setColumnCount(cols)
        table.setVerticalHeaderLabels(statuses)
        table.setHorizontalHeaderLabels(robot_names)

        for r in range(rows):
            for c in range(cols):
                item = qt.QTableWidgetItem()
                flags = item.flags()
                item.setFlags(flags ^ core.Qt.ItemIsEditable)
                table.setItem(r, c, item)

        return (widget, table)

    def statusUpdateMonitor(self, column, msg):
        # Ignore battery, mobility, CPU, and disk as that is now being delivered by the
        # health monitoring system.
        isComms = msg.what == StatusUpdate.WHAT_COMMS
        isRSSI = msg.what == StatusUpdate.WHAT_RSSI
        if not isComms and not isRSSI:
            return

        rgb = GREEN
        if msg.severity == StatusUpdate.SEVERITY_WARNING:
            rgb = YELLOW
        elif msg.severity == StatusUpdate.SEVERITY_CRITICAL:
            rgb = RED
        self.updatePanel(msg.what, column, msg.value, rgb)

    def diagnosticsMonitor(self, prefix_columns, diagArray):
        BATT_ROW = 0
        NODES_ROW = 2
        CPU_ROW = 3
        DISK_ROW = 4

        def levelColor(level):
            if level == DiagnosticStatus.WARN:
                return YELLOW
            elif level == DiagnosticStatus.STALE:
                return YELLOW
            elif level == DiagnosticStatus.ERROR:
                return RED
            else:
                return GREEN

        def extractPercentage(value):
            percent = re.search(r"(\d+\.?\d*%)", value)
            if percent != None and len(percent.groups()) > 0:
                return percent.group(0)
            else:
                return value

        def validPrefix(prefix, what):
            if not prefix_columns.has_key(prefix):
                m = "[Robot Status] Unrecognized robot prefix, %s, for %s"
                rospy.logerr(m, prefix, what)
                return False
            return True

        for s in diagArray.status:
            if re.search(r"NodeletAliveness", s.name):
                if not validPrefix(s.hardware_id, s.name):
                    continue

                good = 0
                rgb = levelColor(s.level)
                for v in s.values:
                    if v.value != "False":
                        good += 1
                val = "{0} of {1}".format(good, len(s.values))
                col = prefix_columns[s.hardware_id]
                self.updatePanel(NODES_ROW, col, val, rgb)

            elif re.search(r"CpuDiagnostic", s.name):
                if not validPrefix(s.hardware_id, s.name):
                    continue

                rgb = levelColor(s.level)
                val = "Unknown"
                col = prefix_columns[s.hardware_id]
                for v in s.values:
                    if v.key.lower() == "total":
                        val = extractPercentage(v.value)
                        break
                self.updatePanel(CPU_ROW, col, val, rgb)

            elif re.search(r"DiskDiagnostic", s.name):
                if not validPrefix(s.hardware_id, s.name):
                    continue

                rgb = levelColor(s.level)
                val = "Unknown"
                col = prefix_columns[s.hardware_id]
                for v in s.values:
                    if v.key.lower() == "total":
                        val = extractPercentage(v.value)
                        break
                self.updatePanel(DISK_ROW, col, val, rgb)

            elif re.search(r"BatteryDiagnostic", s.name):
                if not validPrefix(s.hardware_id, s.name):
                    continue

                rgb = levelColor(s.level)
                val = "Unknown"
                col = prefix_columns[s.hardware_id]
                if len(s.values) > 0:
                    val = s.values[0].value
                self.updatePanel(BATT_ROW, col, val, rgb)


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
