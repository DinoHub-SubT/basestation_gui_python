#!/usr/bin/env python

"""
Plugin for displaying the artifact queue (table of collected artifacts)
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebastion or Matt).
"""
import rospy
import rospkg
import threading
import robots
import os
import requests

import python_qt_binding.QtWidgets as qt
import python_qt_binding.QtCore as core
import python_qt_binding.QtGui as gui

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from qt_gui.plugin import Plugin
from PyQt5.QtCore import pyqtSignal
from gui_utils import COLORS

from gui_utils import displaySeconds, COLORS
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from visualization_msgs.msg import InteractiveMarkerFeedback
from basestation_gui_python.msg import (
    GuiMessage,
    Artifact,
    ArtifactUpdate,
    ArtifactVisualizerUpdate,
    ArtifactSubmissionReply,
)

# Table columns -- each entry describes its column width and column name
COLUMNS = [
    (0, "Unique ID"),
    (85, "Robot"),
    (120, "Time"),
    (140, "Category"),
    (70, "Clone"),
    (70, "Delete"),
    (70, "Refine"),
    (70, "Submit"),
]

UNIQUE_ID = 0
ROBOT_NAME = 1
DETECT_TIME = 2
CATEGORY = 3
DUPLICATE = 4
DELETE = 5
REFINE = 6
SUBMIT = 7

FAKE_ROBOT_UUID = "4e21937b-b55e-49d7-8493-129c9f4e112c"


class ArtifactQueuePlugin(Plugin):
    queue_trigger = pyqtSignal(object)
    update_table_trigger = pyqtSignal(object)
    remove_artifact_trigger = pyqtSignal(object)

    def __init__(self, context):
        super(ArtifactQueuePlugin, self).__init__(context)
        self.setObjectName("ArtifactQueuePlugin")
        self.robots = dict()
        self.all_artifacts = dict()
        self.displayed_artifact_id = None
        self.refined_waypoint = [0, 0, 0]

        config = robots.Config()

        for r in config.robots:
            self.robots[r.uuid] = r

        fake = robots.Robot()
        fake.name = "Phantom"
        fake.uuid = FAKE_ROBOT_UUID
        self.robots[fake.uuid] = fake

        d = config.darpa
        self.report_url = "{0}:{1}{2}".format(
            d.score_address, d.score_port, d.score_report_uri
        )
        self.headers = {"Authorization": "Bearer {0}".format(d.score_bearer_token)}
        self.artifact_categories = d.artifact_categories

        self.initPanel(context)

        self.queue_trigger.connect(self.addArtifactMonitor)
        self.update_table_trigger.connect(self.updateQueueTableMonitor)
        self.remove_artifact_trigger.connect(self.removeArtifactMonitor)

        # setup subscribers/publishers
        self.add_new_artifact_pub = rospy.Publisher(
            "/gui/generate_new_artifact_manual", Artifact, queue_size=10
        )  # manually generate a new artifact
        self.message_pub = rospy.Publisher(
            "/gui/message_print", GuiMessage, queue_size=10
        )  # print a message
        self.delete_artifact_pub = rospy.Publisher(
            "/gui/delete_artifact", String, queue_size=10
        )
        self.duplicate_pub = rospy.Publisher(
            "/gui/duplicate_artifact", String, queue_size=10
        )  # duplicate the message we're clicked on
        self.artifact_submit_pub = rospy.Publisher(
            "/gui/submit_artifact", ArtifactSubmissionReply, queue_size=10
        )  # submit artifacts from queue
        self.focus_on_artifact_pub = rospy.Publisher(
            "/gui/focus_on_artifact", String, queue_size=10
        )  # publish the artifact id we selected
        self.update_label_pub = rospy.Publisher(
            "/gui/update_art_label", ArtifactVisualizerUpdate, queue_size=10
        )  # update image update panel to indicate change to artifact
        self.restart_manip_plugin_pub = rospy.Publisher(
            "/gui/disable_confirm_cancel_manip_plugin", Bool, queue_size=10
        )  # disable confirm/cancel sequence in manipulation plugin
        self.update_artifact_info_pub = rospy.Publisher(
            "/gui/update_artifact_info", ArtifactUpdate, queue_size=10
        )  # for if we get more info from robot
        self.refine_marker_pos_pub = rospy.Publisher(
            "/refinement_marker_pos", Point, queue_size=10
        )
        self.refine_marker_off_pub = rospy.Publisher(
            "/refinement_marker_off", Point, queue_size=10
        )

        self.subscriptions = []

        def sub(topic, what, callback):
            s = rospy.Subscriber(topic, what, callback)
            self.subscriptions.append(s)

        sub("/gui/artifact_to_queue", Artifact, self.addArtifact)
        sub("/gui/update_artifact_in_queue", ArtifactUpdate, self.updateQueueTable)
        sub("/gui/remove_artifact_from_queue", String, self.removeArtifact)
        sub("/artifact_refinement/feedback", InteractiveMarkerFeedback, self.onWaypoint)

    def onWaypoint(self, msg):
        """
	Record the movemment of the interactive marker from RViz where 'msg' is the pose
	of the interactive marker for the artifact refinement.
        """
        p = msg.pose.position
        self.refined_waypoint = [p.x, p.y, p.z]

    def initPanel(self, context):
        """Initialize the panel for displaying widgets."""
        widget = qt.QWidget()
        layout = qt.QGridLayout()

        widget.setWindowTitle("Artifact Detections")
        widget.setLayout(layout)
        context.add_widget(widget)

        self.queue_table = qt.QTableWidget()

        headers = [h[1] for h in COLUMNS]
        self.queue_table.setColumnCount(len(headers))
        self.queue_table.setHorizontalHeaderLabels(headers)
        self.queue_table.setColumnHidden(UNIQUE_ID, True)
        self.queue_table.setSortingEnabled(True)
        self.queue_table.cellClicked.connect(self.queueClick)
        self.queue_table.horizontalHeader().setSectionResizeMode(
            CATEGORY, qt.QHeaderView.Stretch
        )
        for idx, col in enumerate(COLUMNS):
            self.queue_table.setColumnWidth(idx, col[0])
        layout.addWidget(self.queue_table, 0, 0, 1, 4)

        submit = qt.QPushButton("Submit All")
        submit.clicked.connect(self.submitAllArtifacts)
        layout.addWidget(submit, 1, 2)

        insert = qt.QPushButton("Add Artifact")
        insert.clicked.connect(self.manuallyAddArtifact)
        layout.addWidget(insert, 1, 3)

    ############################################################################################
    # Functions dealing with updating artifacts in the queue
    ############################################################################################

    def updateQueueTable(self, msg):
        """For proper threading. See function below."""
        self.update_table_trigger.emit(msg)

    def updateQueueTableMonitor(self, msg):
        """
        Update an element in the queue table

        msg is of type ArtifactUpdate containing the info to be changed
        """
        if not isinstance(threading.current_thread(), threading._MainThread):
            rospy.logerr("[Artifact Queue] Not rendering on main thread.")

        # find the artifact row
        row = None
        for potential_row in range(self.queue_table.rowCount()):
            uid = self.queue_table.item(potential_row, UNIQUE_ID).text()
            if uid == msg.unique_id:
                row = potential_row
                break

        if row is None:
            data = "No such artifact, {0}. Artifact not updated."
            g = GuiMessage()
            g.data = data.format(msg.unique_id)
            g.color = g.COLOR_RED
            self.message_pub.publish(g)
            return

        # determine the column to update and the data to update it with
        if msg.update_type == ArtifactUpdate.PROPERTY_CATEGORY:
            col, data = CATEGORY, msg.category
        elif msg.update_type == ArtifactUpdate.PROPERTY_UNREAD:
            col, data = ROBOT_NAME, ""
        else:
            data = "Received update with unknown property, {0}.  Artifact not updated."
            g = GuiMessage()
            g.data = data.format(msg.update_type)
            g.color = g.COLOR_RED
            self.message_pub.publish(g)
            return

        # actually go and change the appropriate cell in the table
        rows = self.queue_table.rowCount()
        cols = self.queue_table.columnCount()
        if (row > rows or col > cols) and type(data) == str:
            g = GuiMessage()
            g.data = "Artifact row in table no longer exists.  Artifact not updated."
            g.color = g.COLOR_RED
            self.message_pub.publish(g)
            return
        elif type(data) == str:
            if col == CATEGORY:
                self.queue_table.item(row, col).setText(data)
                a = self.all_artifacts[msg.unique_id]
                a.category = data
            else:
                # to remove the green background if its the unread element
                self.queue_table.item(row, col).setBackground(gui.QColor(255, 255, 255))
        else:
            g = GuiMessage()
            g.data = "Artifact update with non-string value.  Artifact not updated."
            g.color = g.COLOR_ORANGE
            self.message_pub.publish(g)

    ############################################################################################
    # Functions dealing with queue button presses
    ############################################################################################

    def manuallyAddArtifact(self):
        """For BSM to manually add an artifact."""
        msg = Artifact()
        msg.original_timestamp = -1
        msg.category = self.artifact_categories[0]
        msg.orig_pose.position.x = -0.1
        msg.orig_pose.position.y = -0.1
        msg.orig_pose.position.z = -0.1
        msg.source_robot_id = -1
        msg.artifact_report_id = -1
        msg.robot_uuid = FAKE_ROBOT_UUID
        msg.imgs = []
        self.add_new_artifact_pub.publish(msg)

    def submitAllArtifacts(self):
        """Submit all of the queued artifacts to DARPA."""
        # Skip is for rows of failed submissions.  Since it has failed then
        # that row will be at the top of the table; hence, we skip over it.
        skip = 0
        for row in range(self.queue_table.rowCount()):
            item = self.queue_table.item(skip, UNIQUE_ID)
            uid = item.text()
            err = self.submitArtifact(uid)
            if err == "":
                self.all_artifacts.pop(uid, None)
                self.queue_table.removeRow(item.row())
            else:
                skip += 1
        if skip > 0:
            m = "Failed to submit all artifacts.  Check log file."
            MB = qt.QMessageBox
            MB.critical(None, "Basestation", m, MB.Ok, MB.Ok)

    def submitArtifact(self, unique_id):
        """
        Submits the artifact with given unique_id to DARPA for scoring.  Returns True if
        the submission was successful or if the artifact doesn't exist within the queue.
        """
        if not self.all_artifacts.has_key(unique_id):
            return True
        artifact = self.all_artifacts[unique_id]
        pos = artifact.curr_pose.position
        report = {"type": artifact.category, "x": pos.x, "y": pos.y, "z": pos.z}
        result = ""
        try:
            req = requests.post(self.report_url, headers=self.headers, json=report)
            if req.ok:
                try:
                    resp = req.json()
                    reply = ArtifactSubmissionReply()
                    reply.unique_id = unique_id
                    reply.submission_time_raw = float(resp["run_clock"])
                    reply.artifact_type = str(resp["type"])
                    reply.x = float(resp["x"])
                    reply.y = float(resp["y"])
                    reply.z = float(resp["z"])
                    reply.score_change = int(resp["score_change"])
                    reply.report_status = str(resp["report_status"])
                    reply.http_response = str(req.status_code)
                    reply.http_reason = str(req.reason)
                    self.artifact_submit_pub.publish(reply)
                except ValueError as e:
                    rospy.logerr("[Artifact Queue] JSON encoding error: %s", e)
                    result = str(e)
            else:
                m = "[Artifact Queue] Bad request -- Code %s, Reason: %s"
                rospy.logerr(m, req.status_code, req.reason)
                result = "{0} (Code {1})".format(req.reason, req.status_code)
        except Exception as e:
            rospy.logerr("[Artifact Queue] POST failed: %s", e)
            result = str(e)
        return result

    ############################################################################################
    # Functions adding/removing artifacts from the queue
    ############################################################################################

    def removeArtifact(self, msg):
        """For proper threading. See function below."""
        self.remove_artifact_trigger.emit(msg)

    def removeArtifactMonitor(self, msg):
        """
        Remove an artifact from the queue, i.e. after is has been submitted

        msg is a string of the artifact unique_id to remove
        """
        if not isinstance(threading.current_thread(), threading._MainThread):
            rospy.logerr("[Artifact Queue] Not rendering on main thread.")

        if msg.data == self.displayed_artifact_id:
            # We're trying to delete something we're currently viewing.
            m = ArtifactVisualizerUpdate()
            m.data = ArtifactVisualizerUpdate.DELETE
            self.update_label_pub.publish(m)
        # Find the artifact in the queue table for removal.
        for row in range(self.queue_table.rowCount()):
            if self.queue_table.item(row, UNIQUE_ID).text() == msg.data:
                m = GuiMessage()
                m.data = "Artifact Removed: " + str(msg.data)
                m.color = m.COLOR_GREEN
                self.message_pub.publish(m)
                self.queue_table.removeRow(self.queue_table.item(row, 0).row())
                self.all_artifacts.pop(msg.data, None)
                return

    def addArtifact(self, msg):
        """For proper threading. See function below."""
        self.queue_trigger.emit(msg)

    def addArtifactMonitor(self, msg):
        """
        Draw something on the gui in this function

        msg is a custom Artifact message containing info about the artifact
        """
        if not isinstance(threading.current_thread(), threading._MainThread):
            rospy.logerr("[Artifact Queue] Not rendering on main thread.")

        self.all_artifacts[msg.unique_id] = msg
        self.queue_table.setSortingEnabled(False)  # to avoid corrupting the table
        self.queue_table.insertRow(self.queue_table.rowCount())

        robot = self.robots[msg.robot_uuid]
        row = self.queue_table.rowCount() - 1
        disp_time = displaySeconds(float(msg.time_from_robot))

        row_data = ["--"] * self.queue_table.columnCount()
        row_data[ROBOT_NAME] = robot.name
        row_data[DETECT_TIME] = disp_time
        row_data[CATEGORY] = msg.category
        row_data[UNIQUE_ID] = msg.unique_id

        for col, val in enumerate(row_data):
            if col == DETECT_TIME:
                colon = disp_time.find(":")
                val = float(disp_time[:colon]) * 60 + float(disp_time[colon + 1 :])
                item = NumericItem(str(disp_time))
                item.setData(core.Qt.UserRole, val)
            else:  # if we're not dealing with a display time
                item = NumericItem(str(val))
                item.setData(core.Qt.UserRole, val)
            self.queue_table.setItem(row, col, item)

        def remove_row():
            for row in range(self.queue_table.rowCount()):
                item = self.queue_table.item(row, UNIQUE_ID)
                if item.text() == msg.unique_id:
                    self.queue_table.removeRow(item.row())
                    self.all_artifacts.pop(msg.unique_id, None)
                    break

        def delete():
            MB = qt.QMessageBox
            answer = MB.warning(
                None,
                "Basestation",
                "Really delete artifact?  This can not be undone.",
                buttons=MB.Yes | MB.Cancel,
                defaultButton=MB.Yes,
            )
            if answer == MB.Cancel:
                return
            s = String()
            s.data = msg.unique_id
            self.delete_artifact_pub.publish(s)
            remove_row()

        def duplicate():
            s = String()
            s.data = msg.unique_id
            self.duplicate_pub.publish(s)

        def refine():
            p = msg.curr_pose.position
            d = RefineDialog(self.artifact_categories, p.x, p.y, p.z)

            def moveMarker(isChecked):
                if not isChecked:
                    d.xCoordEdit.setText(str(self.refined_waypoint[0]))
                    d.yCoordEdit.setText(str(self.refined_waypoint[1]))
                    d.zCoordEdit.setText(str(self.refined_waypoint[2]))
                    d.showMarkerButton.setText("Show RViz Marker")
                    self.refine_marker_off_pub.publish(Point(0, 0, 0))
                    return
                # Set the refinfed_waypoint immediately in case the user
                # does not move the marker in RViz, thus not calling the
                # onWaypoint handler, and then unchecks the 'Show Marker'
                # button.  Not doing this will set the _CoordEdit boxes
                # to whatever junk is currently in refined_waypoint.
                x = float(d.xCoordEdit.text())
                y = float(d.yCoordEdit.text())
                z = float(d.zCoordEdit.text())
                self.refined_waypoint = [x, y, z]
                self.refine_marker_pos_pub.publish(Point(p.x, p.y, p.z))
                d.showMarkerButton.setText("Move RViz Marker")

            d.showMarkerButton.clicked.connect(moveMarker)
            d.categoryComboBox.setCurrentText(msg.category)
            d.exec_()
            if d.result() == 0:
                return

            msg.category = d.categoryComboBox.currentText()
            p.x = float(d.xCoordEdit.text())
            p.y = float(d.yCoordEdit.text())
            p.z = float(d.zCoordEdit.text())
            u = ArtifactUpdate()
            u.unique_id = msg.unique_id
            u.update_type = u.PROPERTY_CATEGORY
            u.category = msg.category
            u.curr_pose.position.x = p.x
            u.curr_pose.position.y = p.y
            u.curr_pose.position.z = p.z
            self.update_artifact_info_pub.publish(u)

        def submit():
            MB = qt.QMessageBox
            answer = MB.warning(
                None,
                "Basestation",
                "Really submit artifact?",
                buttons=MB.Yes | MB.Cancel,
                defaultButton=MB.Yes,
            )
            if answer == MB.Cancel:
                return
            err = self.submitArtifact(msg.unique_id)
            if err == "":
                remove_row()
            else:
                MB.critical(
                    None,
                    "Basestation",
                    "Artifact submission failed: " + err,
                    MB.Ok,
                    MB.Ok,
                )

        def add_btn(column, theme, callback, toolTip):
            b = qt.QToolButton()
            b.setIcon(gui.QIcon.fromTheme(theme))
            b.clicked[bool].connect(callback)
            b.setToolTip(toolTip)
            self.queue_table.setCellWidget(row, column, b)

        add_btn(DUPLICATE, "gtk-copy", duplicate, "Duplicate artifact")
        add_btn(DELETE, "gtk-delete", delete, "Remove artifact from table")
        add_btn(REFINE, "stock_refresh", refine, "Move artifact to a new position")
        add_btn(SUBMIT, "go-home", submit, "Submit artifact to DARPA for scoring")

        # color the unread green
        self.queue_table.item(row, ROBOT_NAME).setBackground(gui.QColor(0, 255, 0))

        # make the cells not editable and make the text centered
        for i in range(self.queue_table.columnCount()):
            if self.queue_table.item(row, i) is not None:
                f = core.Qt.ItemIsSelectable | core.Qt.ItemIsEnabled
                self.queue_table.item(row, i).setFlags(f)
                self.queue_table.item(row, i).setTextAlignment(Qt.AlignHCenter)

        self.queue_table.setSortingEnabled(True)  # to avoid corrupting the table

    ############################################################################################
    # Misc. functions
    ############################################################################################

    def queueClick(self, row, col):
        """
        An artifact was clicked on the artifact queue

        row, col define the coordinates (ints) of the cells that was clicked on
        """
        self.queue_table.selectRow(row)
        # So we know what's currently being displayed.
        self.displayed_artifact_id = self.queue_table.item(row, UNIQUE_ID).text()
        # Publish that we have selected this artifact.
        msg = String()
        msg.data = self.displayed_artifact_id
        self.focus_on_artifact_pub.publish(msg)
        # Remove the artifact update message from the image visualizer plugin if it is
        # still visible.
        msg = ArtifactVisualizerUpdate()
        msg.data = ArtifactVisualizerUpdate.HIDE
        self.update_label_pub.publish(msg)
        # Dectivate the proper buttons in the manipulation panel, we're viewing another
        # artifact, so restart the Confirm/Cancel sequence.
        self.restart_manip_plugin_pub.publish(Bool(True))
        # remove the unread indicator from the last column
        msg = ArtifactUpdate()
        msg.unique_id = self.displayed_artifact_id
        msg.update_type = ArtifactUpdate.PROPERTY_UNREAD
        self.updateQueueTable(msg)

    def shutdown_plugin(self):
        for s in self.subscriptions:
            s.unregister()


class NumericItem(qt.QTableWidgetItem):
    """
    Class which overwrites a pyqt table widget item in order to allow
    for better sorting (e.g. '2'<'100').
    """

    def __lt__(self, other):
        return self.data(core.Qt.UserRole) < other.data(core.Qt.UserRole)


class RefineDialog(qt.QDialog):
    def __init__(self, categories, xPos, yPos, zPos):
        super(RefineDialog, self).__init__()
        bs = rospkg.RosPack().get_path("basestation_gui_python")
        ui = os.path.join(bs, "resources", "refine_artifact.ui")
        loadUi(ui, self, {})
        self.setStyleSheet("QPushButton:checked {" + COLORS.BLUE + "}")
        self.categoryComboBox.addItems(categories)
        self.xCoordEdit.setText(str(xPos))
        self.yCoordEdit.setText(str(yPos))
        self.zCoordEdit.setText(str(zPos))
