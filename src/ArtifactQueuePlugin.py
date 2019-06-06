#!/usr/bin/env python

"""
Plugin for displaying the artifact queue (table of collected artifacts)
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebastion or Matt).
"""
import rospy
import threading
import robots

from qt_gui.plugin import Plugin
import python_qt_binding.QtWidgets as qt
import python_qt_binding.QtCore as core
import python_qt_binding.QtGui as gui

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QWidget
from PyQt5.QtCore import pyqtSignal

from gui_utils import displaySeconds, COLORS
from std_msgs.msg import String, Bool
from basestation_gui_python.msg import GuiMessage, Artifact, ArtifactUpdate


class ArtifactQueuePlugin(Plugin):
    queue_trigger = pyqtSignal(object)  # to keep the drawing on the proper thread
    archive_artifact_trigger = pyqtSignal()
    update_table_trigger = pyqtSignal(object)
    submit_all_artifacts_trigger = pyqtSignal()
    remove_artifact_from_queue_trigger = pyqtSignal(object)

    def __init__(self, context):
        super(ArtifactQueuePlugin, self).__init__(context)
        self.setObjectName("ArtifactQueuePlugin")
        self.artifact_categories = robots.Config().darpa.artifact_categories
        self.displayed_artifact_id = None
        self.column_headers = [
            "Robot\nNum",
            "Detect\nTime",
            "   Category   ",
            "Unread",
            "Unique ID",
        ]

        self.col_robot_num = 0
        self.col_time = 1
        self.col_category = 2
        self.col_unread = 3
        self.col_unique_id = 4

        self.initPanel(context)  

        self.queue_trigger.connect(self.addArtifactToQueueMonitor)
        self.archive_artifact_trigger.connect(self.confirmArchiveArtifactMonitor)
        self.update_table_trigger.connect(self.updateQueueTableMonitor)
        self.submit_all_artifacts_trigger.connect(self.submitAllQueuedArtifactsMonitor)
        self.remove_artifact_from_queue_trigger.connect(
            self.removeArtifactFromQueueMonitor
        )

        # setup subscribers/publishers
        self.add_new_artifact_pub = rospy.Publisher(
            "/gui/generate_new_artifact_manual", Artifact, queue_size=10
        )  # manually generate a new artifact
        self.message_pub = rospy.Publisher(
            "/gui/message_print", GuiMessage, queue_size=10
        )  # print a message
        self.archive_artifact_pub = rospy.Publisher(
            "/gui/archive_artifact", String, queue_size=10
        )  # archive a message
        self.duplicate_pub = rospy.Publisher(
            "/gui/duplicate_artifact", String, queue_size=10
        )  # duplicate the message we're clicked on
        self.artifact_submit_pub = rospy.Publisher(
            "/gui/submit_artifact_from_queue", String, queue_size=10
        )  # submit artifacts from queue
        self.focus_on_artifact_pub = rospy.Publisher(
            "/gui/focus_on_artifact", String, queue_size=10
        )  # publish the artifact id we selected
        self.update_label_pub = rospy.Publisher(
            "/gui/update_art_label", String, queue_size=10
        )  # update image update panel to indicate change to artifact
        self.restart_manip_plugin_pub = rospy.Publisher(
            "/gui/disable_confirm_cancel_manip_plugin", Bool, queue_size=10
        )  # disable confirm/cancel sequence in manipulation plugin

        self.queue_sub = rospy.Subscriber(
            "/gui/artifact_to_queue", Artifact, self.addArtifactToQueue
        )  # some plugin wants us to add an artifact
        self.update_sub = rospy.Subscriber(
            "/gui/update_artifact_in_queue", ArtifactUpdate, self.updateQueueTable
        )  # an artifact's property has changed. just update the queue accordingly
        self.remove_sub = rospy.Subscriber(
            "/gui/remove_artifact_from_queue", String, self.removeArtifactFromQueue
        )  # some plugin wants us to remove a plugin
        self.submit_from_manip_sub = rospy.Subscriber(
            "/gui/submit_artifact_from_manip_plugin", String, self.artifactSubmitted
        )  # the manipulation plugin has submitted an artifact, we need to keep
        # track of this happening

    def initPanel(self, context):
        """Initialize the panel for displaying widgets."""
        widget = QWidget()
        layout = qt.QGridLayout()

        context.add_widget(widget)
        widget.setLayout(layout)

        label = qt.QLabel()
        label.setText("ARTIFACT QUEUE")
        label.setAlignment(Qt.AlignCenter)
        layout.addWidget(label, 0, 0, 1, 3)

        self.sort_button = qt.QPushButton("Sort by time")
        self.sort_button.setCheckable(True)
        self.sort_button.toggle()  # start with it sorting the table
        layout.addWidget(self.sort_button, 1, 0)

        insert = qt.QPushButton("Add artifact")
        insert.clicked.connect(self.manuallyAddArtifact)
        layout.addWidget(insert, 1, 1)

        duplicate = qt.QPushButton("Duplicate artifact")
        duplicate.clicked.connect(self.duplicateArtifact)
        layout.addWidget(duplicate, 1, 2)

        archive = qt.QPushButton("Archive artifact")
        archive.clicked.connect(self.archiveArtifact)
        layout.addWidget(archive, 2, 0)

        self.confirm_button = qt.QPushButton("Confirm")
        self.confirm_button.setCheckable(True)
        self.confirm_button.clicked.connect(self.confirmArchiveArtifact)
        self.confirm_button.setStyleSheet(COLORS.GRAY)
        self.confirm_button.setEnabled(False)
        layout.addWidget(self.confirm_button, 2, 2)

        self.cancel_button = qt.QPushButton("Cancel")
        self.cancel_button.setCheckable(True)
        self.cancel_button.clicked.connect(self.cancelArchiveArtifact)
        self.cancel_button.setStyleSheet(COLORS.GRAY)
        self.cancel_button.setEnabled(False)
        layout.addWidget(self.cancel_button, 2, 1)

        submit = qt.QPushButton("Submit all queued artifacts!")
        submit.clicked.connect(self.submitAllQueuedArtifacts)
        layout.addWidget(submit, 3, 0, 1, 3)

        self.queue_table = qt.QTableWidget()
        self.queue_table.horizontalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)

        self.queue_table.setColumnCount(len(self.column_headers))  # set column count
        self.queue_table.setHorizontalHeaderLabels(
            self.column_headers
        )  # make the column headers

        self.queue_table.setColumnHidden(self.col_unique_id, True)  # hide the unique_id
        self.queue_table.setSortingEnabled(True)  # make sortable

        # add click listener
        self.queue_table.cellClicked.connect(self.queueClick)
        layout.addWidget(self.queue_table, 4, 0, 1, 3)

    ############################################################################################
    # Functions dealing with updating artifacts in the queue
    ############################################################################################

    def updateQueueTable(self, msg):
        """
		For proper threading. See function below. 
		"""
        self.update_table_trigger.emit(msg)

    def updateQueueTableMonitor(self, msg):
        """
		Update an element in the queue table

		msg is of type ArtifactUpdate containing the info to be changed
		"""

        # check that threading is working properly
        if not isinstance(threading.current_thread(), threading._MainThread):
            print(
                "Drawing on the message panel not guarenteed to be on the proper thread"
            )

        # find the artifact row
        row = None
        for potential_row in range(self.queue_table.rowCount()):
            if (
                self.queue_table.item(potential_row, self.col_unique_id).text()
                == msg.unique_id
            ):
                row = potential_row
                break

        if row is None:
            update_msg = GuiMessage()
            update_msg.data = "Could not find artifact with proper unique_id in table. Artifact not updated in queue"
            update_msg.color = update_msg.COLOR_RED
            self.message_pub.publish(update_msg)

            return

        # determine the column to update and the data to update it with
        if msg.update_type == ArtifactUpdate.PROPERTY_CATEGORY:
            col, data = self.col_category, msg.category

        # elif msg.update_type == ArtifactUpdate.PROPERTY_PRIORITY:
        #     col, data = 1, msg.priority

        elif msg.update_type == ArtifactUpdate.PROPERTY_UNREAD:
            col, data = self.col_unread, ""

        else:

            update_msg = GuiMessage()
            update_msg.data = "We received an update message of unknown type or of type pose. Artifact queue not updated"
            update_msg.color = update_msg.COLOR_RED
            self.message_pub.publish(update_msg)
            return

        # actually go and change the appropriate cell in the table
        if (
            row > self.queue_table.rowCount() or col > self.queue_table.columnCount()
        ) and type(data) == str:

            update_msg = GuiMessage()
            update_msg.data = "Something got misaligned in the artifact queue table and we are trying to update a cell that no longer exists. Artifact queue not updated."
            update_msg.color = update_msg.COLOR_RED
            self.message_pub.publish(update_msg)
            return

        elif type(data) == str:

            self.queue_table.item(row, col).setText(data)

            self.queue_table.item(row, col).setBackground(
                gui.QColor(255, 255, 255)
            )  # to remove the green background if its the unread element

        else:
            msg = GuiMessage()
            msg.data = "Tried to update queue table with non-string value. Therefore table not updated"
            msg.color = msg.COLOR_ORANGE
            self.message_pub.publish(msg)

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
        self.add_new_artifact_pub.publish(msg)

    def duplicateArtifact(self):
        """Duplicate the artifact(s) that is being clicked on."""
        rows_selected = self.getSelectedRowIndices()

        if len(rows_selected) == 0:
            msg = GuiMessage()
            msg.data = "No artifact(s) selected, did not duplicate anything"
            msg.color = msg.COLOR_ORANGE
            self.message_pub.publish(msg)
        # make the confirm/delete buttons pressable and the correct color
        else:
            msg = String()
            for row in rows_selected:
                msg.data = self.queue_table.item(row, self.col_unique_id).text()
                self.duplicate_pub.publish(msg)

    def archiveArtifact(self):
        """
        Archive artifact. Currently, this means it never shows back up
        in the gui. Adds it to a list of archived artifacts in the
        artifact handler
        """
        # confirm we've actually selected an artifact
        rows_selected = self.getSelectedRowIndices()

        if len(rows_selected) == 0:
            msg = GuiMessage()
            msg.data = "No artifact(s) selected, did not delete anything"
            msg.color = msg.COLOR_ORANGE
            self.message_pub.publish(msg)
        # make the confirm/delete buttons pressable and the correct color
        else:
            self.cancel_button.setStyleSheet(COLORS.RED)
            self.cancel_button.setEnabled(True)
            self.confirm_button.setStyleSheet(COLORS.GREEN)
            self.confirm_button.setEnabled(True)

    def getSelectedRowIndices(self):
        """
        Returns the indice(s) of the selected rows in the queue table.
        Always returns a list, unlike self.queue_table.selectionModel().selectedRows()
        """
        if self.queue_table.rowCount() == 0:
            return []
        return [a.row() for a in self.queue_table.selectionModel().selectedRows()]

    def confirmArchiveArtifact(self):
        """For proper threading. See function below."""
        self.archive_artifact_trigger.emit()

    def confirmArchiveArtifactMonitor(self):
        """
        A deletion of an artifact has been confirmed. Remove the artifact from the
        queue.
        """
        if not isinstance(threading.current_thread(), threading._MainThread):
            rospy.logerr("[Artifact Queue] Not rendering on main thread.")

        # Needs to in reverse order so we don't try to access a row that no longer
        # exists.
        rows_selected = self.getSelectedRowIndices()
        for row in sorted(rows_selected, reverse=True):
            # if we're archiving something we're viewing, reset the id we're viewing
            if (
                self.queue_table.item(row, self.col_unique_id).text()
                == self.displayed_artifact_id
            ):
                self.displayed_artifact_id = None

            # delete it from the ArtifactHandler book-keeping
            handler_msg = String()
            handler_msg.data = self.queue_table.item(row, self.col_unique_id).text()
            self.archive_artifact_pub.publish(handler_msg)  # only send the unique ID

            # delete it from the queue table
            msg = String()
            msg.data = self.queue_table.item(row, self.col_unique_id).text()
            self.removeArtifactFromQueue(msg)

            # reset the confirm/cancel buttons
            self.cancel_button.setStyleSheet(COLORS.GRAY)
            self.cancel_button.setEnabled(False)
            self.confirm_button.setStyleSheet(COLORS.GRAY)
            self.confirm_button.setEnabled(False)

    def cancelArchiveArtifact(self):
        """
        Do not actually archive the artifact. Leave it as-is.
        Just reset the confirm/cancel buttons
        """
        self.cancel_button.setStyleSheet(COLORS.GRAY)
        self.cancel_button.setEnabled(False)
        self.confirm_button.setStyleSheet(COLORS.GRAY)
        self.confirm_button.setEnabled(False)

    def submitAllQueuedArtifacts(self):
        """For threading properly. See function below."""
        self.submit_all_artifacts_trigger.emit()

    def submitAllQueuedArtifactsMonitor(self):
        """Submit all of the queued artifacts to DARPA."""
        if not isinstance(threading.current_thread(), threading._MainThread):
            rospy.logerr("[Artifact Queue] Not rendering on main thread.")

        msg = String()
        for row in range(self.queue_table.rowCount()):
            msg.data = self.queue_table.item(row, self.col_unique_id).text()
            self.artifact_submit_pub.publish(msg)
        # remove all of the rows from the table
        self.queue_table.setRowCount(0)

    ############################################################################################
    # Functions adding/removing artifacts from the queue
    ############################################################################################

    def removeArtifactFromQueue(self, msg):
        """For proper threading. See function below."""
        self.remove_artifact_from_queue_trigger.emit(msg)

    def removeArtifactFromQueueMonitor(self, msg):
        """
        Remove an artifact from the queue, i.e. after is has been submitted

        msg is a string of the artifact unique_id to remove
        """
        if not isinstance(threading.current_thread(), threading._MainThread):
            rospy.logerr("[Artifact Queue] Not rendering on main thread.")

        if msg.data == self.displayed_artifact_id:
            # We're trying to delete something we're currently viewing.
            m = String()
            m.data = "Artifact deleted"
            self.update_label_pub.publish(m)
        # Find the artifact in the queue table for removal.
        for row in range(self.queue_table.rowCount()):
            if self.queue_table.item(row, self.col_unique_id).text() == msg.data:
                
                m = GuiMessage()
                m.data = "Artifact removed:" + str(msg.data)
                m.color = m.COLOR_GREEN
                self.message_pub.publish(m)
                self.queue_table.removeRow(self.queue_table.item(row, 0).row())

                return

        # If we get to this point, we did not find the artifact to delete.
        m = GuiMessage()
        m.data = "Artifact(ID {0}) not found and not removed.".format(msg.data)
        m.color = update_msg.COLOR_RED
        self.message_pub.publish(m)

    def addArtifactToQueue(self, msg):
        """For proper threading. See function below."""
        self.queue_trigger.emit(msg)

    def addArtifactToQueueMonitor(self, msg):
        """
        Draw something on the gui in this function

        msg is a custom Artifact message containing info about the artifact
        """
        if not isinstance(threading.current_thread(), threading._MainThread):
            rospy.logerr("[Artifact Queue] Not rendering on main thread.")

        self.queue_table.setSortingEnabled(False)  # to avoid corrupting the table
        self.queue_table.insertRow(self.queue_table.rowCount())

        row = self.queue_table.rowCount() - 1
        disp_time = displaySeconds(float(msg.time_from_robot))

        row_data = ["--"] * self.queue_table.columnCount()
        row_data[self.col_robot_num] = msg.source_robot_id
        row_data[self.col_time] = disp_time
        row_data[self.col_category] = msg.category
        row_data[self.col_unread] = "!"
        row_data[self.col_unique_id] = msg.unique_id

        for col, val in enumerate(row_data):

            if col == self.col_time:

                colon = disp_time.find(":")
                val = float(disp_time[:colon]) * 60 + float(disp_time[colon + 1 :])
                item = NumericItem(str(disp_time))
                item.setData(core.Qt.UserRole, val)
            else:  # if we're not dealing with a display time
                item = NumericItem(str(val))
                item.setData(core.Qt.UserRole, val)
            self.queue_table.setItem(row, col, item)

        # color the unread green
        self.queue_table.item(row, self.col_unread).setBackground(gui.QColor(0, 255, 0))

        # make the cells not editable and make the text centered
        for i in range(self.queue_table.columnCount()):
            if self.queue_table.item(row, i) is not None:
                f = core.Qt.ItemIsSelectable | core.Qt.ItemIsEnabled
                self.queue_table.item(row, i).setFlags(f)
                self.queue_table.item(row, i).setTextAlignment(Qt.AlignHCenter)

        self.queue_table.setSortingEnabled(True)  # to avoid corrupting the table

        # if the sort button is pressed, sort the incoming artifacts
        if self.sort_button.isChecked():
            self.queue_table.sortItems(2, core.Qt.DescendingOrder)
            self.queue_table.viewport().update()

    ############################################################################################
    # Misc. functions
    ############################################################################################

    def artifactSubmitted(self, msg):
        """
        An artifact was just submitted. IF its the one we were viewing, reset the displayed artifact id

        msg is a String with the unique id of the artifact
        """
        if self.displayed_artifact_id == msg.data:
            self.displayed_artifact_id = None

    def queueClick(self, row, col):
        """
        An artifact was clicked on the artifact queue

        row, col define the coordinates (ints) of the cells that was clicked on
        """
        # select the whole row. for visualization mostly
        self.queue_table.selectRow(row)

        # so we know what's currently being displayed
        self.displayed_artifact_id = self.queue_table.item(
            row, self.col_unique_id
        ).text()

        # publish that we have selected this artifact
        msg = String()
        msg.data = self.displayed_artifact_id
        self.focus_on_artifact_pub.publish(msg)

        # remove the artifact update message from the image visualizer plugin
        # if it is still visible
        msg = String()
        msg.data = "hide"
        self.update_label_pub.publish(msg)

        # dectivate the proper buttons in the manipulation panel, we're viewing another artifact,
        # so restart the Confirm/Cancel sequence.
        self.restart_manip_plugin_pub.publish(Bool(True))

        # remove the unread indicator from the last column
        msg = ArtifactUpdate()
        msg.unique_id = self.displayed_artifact_id
        msg.update_type = ArtifactUpdate.PROPERTY_UNREAD
        self.updateQueueTable(msg)


    def shutdown_plugin(self):
        self.submit_from_manip_sub.unregister()
        self.queue_sub.unregister()
        self.remove_sub.unregister()
        self.update_sub.unregister()


class NumericItem(qt.QTableWidgetItem):
    """
    Class which overwrites a pyqt table widget item in order to allow
    for better sorting (e.g. '2'<'100').
    """

    def __lt__(self, other):
        return self.data(core.Qt.UserRole) < other.data(core.Qt.UserRole)
