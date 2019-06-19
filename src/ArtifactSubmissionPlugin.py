#!/usr/bin/env python

"""
Skeleton code to make a new plugin
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebation or Matt).
"""
import rospy
import rospkg
import threading

import python_qt_binding.QtWidgets as qt
import python_qt_binding.QtCore as core
import python_qt_binding.QtGui as gui

from python_qt_binding.QtCore import Qt
from qt_gui.plugin import Plugin
from PyQt5.QtCore import pyqtSignal
from gui_utils import displaySeconds

from std_msgs.msg import String
from basestation_gui_python.msg import GuiMessage, DarpaStatus, ArtifactSubmissionReply


class ArtifactSubmissionPlugin(Plugin):
    # to keep the drawing on the proper thread
    submission_reply_trigger = pyqtSignal(object)

    def __init__(self, context):
        super(ArtifactSubmissionPlugin, self).__init__(context)
        self.setObjectName("ArtifactSubmissionPlugin")
        self.column_headers = ["Category", "Time", "x/y/z", "Response"]
        self.initPanel(context)
        self.submission_reply_trigger.connect(self.submissionReplyMonitor)

        # contains info from darpa about the artifact submission
        self.submission_reply_sub = rospy.Subscriber(
            "/gui/submission_reply", ArtifactSubmissionReply, self.submissionReply
        )

    def initPanel(self, context):
        """Initialize the panel for displaying widgets."""
        # define the overall widget
        self.submission_reply_widget = qt.QWidget()
        self.submission_reply_layout = qt.QGridLayout()

        self.submission_reply_widget.setWindowTitle("Artifact Submissions")
        context.add_widget(self.submission_reply_widget)

        # add the sort on/off button
        self.submission_reply_table_sort_button = qt.QPushButton("Sort by time")
        self.submission_reply_table_sort_button.setCheckable(
            True
        )  # a button pressed will stay pressed, until unclicked
        self.submission_reply_table_sort_button.toggle()  # start with it sorting the table

        self.submission_reply_layout.addWidget(
            self.submission_reply_table_sort_button, 0, 0
        )

        self.submission_reply_table = qt.QTableWidget()

        # resize the cells to fill the widget
        self.submission_reply_table.horizontalHeader().setSectionResizeMode(
            qt.QHeaderView.Stretch
        )
        # self.arthist_table.verticalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)

        self.submission_reply_table.setColumnCount(
            len(self.column_headers)
        )  # set column count
        self.submission_reply_table.setHorizontalHeaderLabels(self.column_headers)
        self.submission_reply_table.setSortingEnabled(True)

        tooltip_font = gui.QFont()
        tooltip_font.setPointSize(13)

        self.submission_reply_layout.addWidget(self.submission_reply_table)
        self.submission_reply_widget.setLayout(self.submission_reply_layout)

    def submissionReply(self, msg):
        """For proper threading. See function below."""
        self.submission_reply_trigger.emit(msg)

    def submissionReplyMonitor(self, msg):
        """
        Populate the table with infromation about an artifact that was submitted.

        msg is a custom ArtifactSubmissionReply message containing info from darpa about the artifact we submitted
        """
        if not isinstance(threading.current_thread(), threading._MainThread):
            rospy.logerr("[Artifact Submission] Not rendering on main thread.")

        # make a table item to add
        try:
            submission_time = displaySeconds(float(msg.submission_time_raw))
        except:
            submission_time = "99:99"

        if msg.score_change == 0:
            submission_correct = "False"
            submission_color = gui.QColor(220, 0, 0)
        else:
            submission_correct = "True"
            submission_color = gui.QColor(0, 220, 0)

        response_item = qt.QTableWidgetItem("Info")
        response_item.setToolTip(
            "DARPA response: "
            + str(msg.report_status)
            + "\nHTTP Response: "
            + str(msg.http_response)
            + str(msg.http_reason)
            + "\nSubmission Correct? "
            + submission_correct
        )
        response_item.setBackground(submission_color)

        # add the item to the table
        self.submission_reply_table.setSortingEnabled(False)
        self.submission_reply_table.insertRow(self.submission_reply_table.rowCount())
        row = self.submission_reply_table.rowCount() - 1

        row_data = [
            msg.artifact_type,
            submission_time,
            str(int(msg.x)) + "/" + str(int(msg.y)) + "/" + str(int(msg.z)),
        ]

        for col, val in enumerate(row_data):
            if col == 1:
                colon = submission_time.find(":")
                stime = float(submission_time[colon + 1 :])
                val = float(submission_time[:colon]) * 60 + stime
                item = NumericItem(str(submission_time))
                item.setData(core.Qt.UserRole, val)
            else:  # if we're not dealing with a display time
                item = NumericItem(str(val))
                item.setData(core.Qt.UserRole, val)
            # make it non-editable
            item.setFlags(core.Qt.ItemIsSelectable | core.Qt.ItemIsEnabled)
            item.setTextAlignment(Qt.AlignHCenter)
            self.submission_reply_table.setItem(row, col, item)

        # add the response item (has tool-tip with info about the submission) we don't want this to be a NumericItem
        response_item.setFlags(core.Qt.ItemIsSelectable | core.Qt.ItemIsEnabled)
        self.submission_reply_table.setItem(row, 3, response_item)
        self.submission_reply_table.setSortingEnabled(True)

        # if the sort button is pressed, sort the incoming artifacts
        if self.submission_reply_table_sort_button.isChecked():
            self.submission_reply_table.sortItems(1, core.Qt.DescendingOrder)
            self.submission_reply_table.viewport().update()

    def shutdown_plugin(self):
        self.submission_reply_sub.unregister()


class NumericItem(qt.QTableWidgetItem):
    """
	Class which overwrites a pyqt table widget item in order to allow for better sorting (e.g. '2'<'100')
	"""

    def __lt__(self, other):
        return self.data(core.Qt.UserRole) < other.data(core.Qt.UserRole)
