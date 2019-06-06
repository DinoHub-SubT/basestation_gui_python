#!/usr/bin/env python

"""
Plugin for buttons pertaining to robot commands.

Contact: Bob DeBortoli (debortor@oregonstate.edu)
Copyright 2019: Carnegie Mellon University / Oregon State University

This code is proprietary to the CMU SubT challenge. Do not share or distribute without
express permission of a project lead (Sebastion or Matt).
"""

import rospy
import rospkg
import threading
import robots

import python_qt_binding.QtWidgets as qt

from qt_gui.plugin import Plugin
from PyQt5.QtCore import pyqtSignal
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy, QTabWidget

from basestation_gui_python.msg import GuiMessage, RadioMsg, NineHundredRadioMsg
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import InteractiveMarkerFeedback

from gui_utils import COLORS

################################################################################
#
#    Helper functions for publishing radio messages.
#
################################################################################


def radio900(pub, what, exec_id):
    r9 = NineHundredRadioMsg()
    r9.message_type = what
    r9.recipient_robot_id = exec_id
    pub.publish(r9)


def radio(pub, msg_type, what, exec_id):
    rm = RadioMsg()
    rm.data = what
    rm.recipient_robot_id = exec_id
    rm.message_type = msg_type
    pub.publish(rm)


def radioStop(pub, what, exec_id):
    radio(pub, RadioMsg.MESSAGE_TYPE_ESTOP, what, exec_id)


################################################################################
#
#    Plugin for building tabbed panel containing buttons to command the
#    various robots.
#
################################################################################


class RobotCommandPlugin(Plugin):
    rob_command_trigger = pyqtSignal(object)  # to keep the drawing on the proper thread

    def __init__(self, context):
        super(RobotCommandPlugin, self).__init__(context)
        self.setObjectName("RobotCommandPlugin")
        self.config = robots.Config()
        self.timer_buttons = []
        self.positions = dict()
        self.waypoints = dict()
        self.waypt_robot = None

        def pub(to, what, size):
            return rospy.Publisher(to, what, queue_size=size)

        self.radio_pub = pub("/from_gui", RadioMsg, 50)
        self.radio_900_pub = pub("/ros_to_teensy", NineHundredRadioMsg, 50)
        self.gui_message_pub = pub("/gui/message_print", GuiMessage, 10)
        self.waypoint_marker_on_pub = pub("/define_waypoint_marker_pos", Point, 50)
        self.waypoint_marker_off_pub = pub("/define_waypoint_marker_off", Point, 50)
        self.subs = [
            rospy.Subscriber(
                "/define_waypoint/feedback", InteractiveMarkerFeedback, self.onWaypoint
            )
        ]
        for r in self.config.robots:
            t = "/{0}/{1}".format(r.topic_prefix, r.topics["odometry"])
            self.subs.append(rospy.Subscriber(t, Odometry, self.onOdometry, r))
        context.add_widget(self.createPanel())

    def shutdown_plugin(self):
        for s in self.subs:
            s.unregister()
        for b in self.timer_buttons:
            b.uncheck()

    def publishWaypoint(self, robot):
        """Callback to publish a waypoint for the robot to move to."""
        if self.waypoints.has_key(robot.uuid):
            w = self.waypoints[robot.uuid]
            d = "{0},{1},{2}".format(w[0], w[1], w[2])
            m = RadioMsg.MESSAGE_TYPE_DEFINE_WAYPOINT
            radio(self.radio_pub, m, d, robot.executive_id)
            # Publish an invalid point to turn off the interactive marker in Rviz.
            self.waypoint_marker_off_pub.publish(Point(-1, -1, -1))
            self.waypt_robot = None
        else:
            m = GuiMessage()
            m.data = "No waypoint received from RViz."
            m.color = m.COLOR_GRAY
            self.gui_message_pub.publish(m)

    def moveWaypoint(self, robot):
        """Callback to have RViz place an interactive marker to define a waypoint."""
        if self.positions.has_key(robot.uuid):
            p = self.positions.get(robot.uuid)
            self.waypt_robot = robot
            self.waypoint_marker_on_pub.publish(Point(p[0], p[1], p[2]))
            return False
        else:
            m = GuiMessage()
            m.data = "No odometry received for {0}.  Check topic.".format(robot.name)
            m.color = m.COLOR_GRAY
            self.gui_message_pub.publish(m)
            return True

    def onWaypoint(self, msg):
        """
	Record the movemment of the interactive marker from RViz where 'msg' is the pose
	of the interactive marker for defining a waypoint.
        """
        if self.waypt_robot == None:
            return
        p = msg.pose.position
        self.waypoints[self.waypt_robot.uuid] = [p.x, p.y, p.z]

    def onOdometry(self, odom, robot):
        """Store the robot's position in order to publish waypoint."""
        p = odom.pose.pose.position
        self.positions[robot.uuid] = [p.x, p.y, p.z]

    def createPanel(self):
        widget = QWidget()
        layout = qt.QGridLayout()
        tabs = QTabWidget()
        label = qt.QLabel()
        rad = self.radio_pub
        rad900 = self.radio_900_pub
        waypt_links = []
        stoppers = []

        label.setText("CONTROL PANEL")
        label.setAlignment(Qt.AlignCenter)
        layout.addWidget(label)
        layout.addWidget(tabs)
        widget.setLayout(layout)

        def make_adjuster(combo_box, exec_id):
            def fn():
                m = RadioMsg.MESSAGE_TYPE_MAX_FLIGHT_TIME
                d = str(float(combo_box.currentText()) * 60)
                radio(self.radio_pub, m, d, exec_id)

            return fn

        MAX_ROWS = 5
        for robot in self.config.robots:
            row, col = 0, 0
            exec_id = robot.executive_id
            grid = qt.QGridLayout()
            tab = QWidget()
            buttons = []
            confirm = ConfirmButton("Confirm", COLORS.GREEN)
            cancel = ConfirmButton("Cancel", COLORS.RED)
            waypt = DefineWaypointButton(robot, self.moveWaypoint, self.publishWaypoint)
            blue = ShowBluetoothButton()
            goHome = ReturnHomeButton(exec_id, rad)
            resend = RadioButton(
                "Re-send Artifacts",
                exec_id,
                rad,
                RadioMsg.MESSAGE_TYPE_RESEND_ALL_ARTIFACTS,
            )

            for w in waypt_links:
                w.link([waypt])
                waypt.link([w])
            waypt_links.append(waypt)

            tab.setLayout(grid)
            tabs.addTab(tab, robot.name)

            if robot.is_aerial:
                hover = HoverButton(exec_id, rad)
                takeoff = TakeoffButton(exec_id, rad, rad900)
                hard = AerialHardEStopButton(confirm, cancel, exec_id, rad, rad900)
                land = LandButton(confirm, cancel, exec_id, rad)
                toComms = ExploreButton("Land in Comms", exec_id, rad)

                stoppers.append(land)
                self.timer_buttons.append(hard)
                hover.link([takeoff, hard, land, goHome])
                takeoff.link([hover, hard, land, goHome])
                hard.link([hover, takeoff, land, goHome])
                land.link([hover, takeoff, hard, goHome])
                goHome.link([hover, takeoff, land, hard])

                commands = [
                    hover,
                    takeoff,
                    land,
                    hard,
                    waypt,
                    goHome,
                    toComms,
                    resend,
                    confirm,
                    cancel,
                ]
            else:
                commands = []
                resume = ResumeButton(exec_id, rad, rad900)
                soft = SoftEStopButton(exec_id, rad)
                hard = HardEStopButton(confirm, cancel, exec_id, rad, rad900)
                toComms = ExploreButton("Return to Comms", exec_id, rad)

                stoppers.append(soft)
                resume.link([soft, hard, goHome])
                soft.link([resume, hard, goHome])
                hard.link([resume, soft, goHome])
                goHome.link([resume, soft, hard])

                commands.extend([resume, soft, hard, waypt, goHome])
                if robot.has_comms:
                    mt = RadioMsg.MESSAGE_TYPE_DROP_COMMS
                    commands.append(RadioButton("Drop Comms", exec_id, rad, mt))
                commands.extend([blue, toComms, resend, confirm, cancel])

            for c in commands:
                buttons.append(c)
                grid.addWidget(c, row, col)
                if row == MAX_ROWS:
                    row = 0
                    col += 1
                else:
                    row += 1

            max_time = (2 * robot.max_travel_time) + 1
            times = [str(n / 2.0) for n in range(0, max_time)]
            box = qt.QComboBox()
            for speed in times:
                box.addItem(speed)
            box.currentTextChanged.connect(make_adjuster(box, exec_id))
            grid.addWidget(box, row, col)

        def stopAll():
            for s in stoppers:
                s.forceStop()

        all_stop = qt.QPushButton("Soft E-Stop All")
        all_stop.setStyleSheet(COLORS.RED)
        all_stop.clicked.connect(stopAll)
        layout.addWidget(all_stop)

        return widget


################################################################################
#
#    Command buttons that execute individual robot commands when clicked.
#
################################################################################


class RadioButton(qt.QPushButton):
    """
    RadioButton is a simple fire and forget button that sends a RadioMsg when clicked.
    """

    def __init__(self, text, exec_id, radio_pub, msg_type):
        super(RadioButton, self).__init__(text)

        def onClick():
            radio(radio_pub, msg_type, "", exec_id)

        self.setStyleSheet("QPushButton:pressed {" + COLORS.GREEN + "}")
        self.clicked.connect(onClick)


class ConfirmButton(qt.QPushButton):
    """
    ConfirmButton is special purpose button to implement "Confirm/Cancel" buttons for
    other actions that need confirmation.  A ConfirmButton can be activated and
    de-activated (i.e. enabled/disabled).  When activated a callback can be set that
    will be called if the user clicks this button.
    """

    def __init__(self, text, color):
        super(ConfirmButton, self).__init__(text)
        self.setEnabled(False)
        self.setStyleSheet(COLORS.GRAY)
        self.color = color
        self.callback = None
        self.clicked.connect(self.execute)

    def execute(self):
        if self.callback != None:
            self.callback()

    def activate(self, callback):
        self.callback = callback
        self.setEnabled(True)
        self.setStyleSheet(self.color)

    def deactivate(self):
        self.callback = None
        self.setEnabled(False)
        self.setStyleSheet(COLORS.GRAY)


class LinkableButton(qt.QPushButton):
    """
    LinkableButton overrides a regular QT button that allows other buttons to be linked
    to this one.  When the linked button is pushed it will deactivate all other linked
    button by setting the checked property to false.
    """

    def __init__(self, text):
        super(LinkableButton, self).__init__(text)
        self.linked = []
        self.setCheckable(True)
        self.setStyleSheet("QPushButton:checked {" + COLORS.RED + "}")
        self.clicked.connect(self.onClick)

    def onClick(self, isChecked):
        if not isChecked:
            self.uncheck()
            return
        self.updateLinks()
        self.execute()

    def updateLinks(self):
        for btn in self.linked:
            btn.setChecked(False)
            btn.updateLink()

    def link(self, buttons):
        """
        Link other checkable buttons that will be un-checked when this button is
        activated.
        """
        self.linked.extend(buttons)

    def uncheck(self):
        """
        Allow derived classes to execute custom logic when the button is unchecked.
        """
        pass

    def updateLink(self):
        """Notify derived classes that one of their linked buttons has been checked."""
        pass

    def execute(self):
        """
        Execute needs to be implemented by derived classes to execute custom behavior
        for when the button is checked.
        """
        raise NotImplementedError()


class LinkableConfirmButton(LinkableButton):
    """
    LinkableConfirmButton is like a LinkableButton but uses confirm and cancel buttons
    before calling execute on its derived class.
    """

    def __init__(self, text, confirm, cancel):
        super(LinkableConfirmButton, self).__init__(text)
        self.confirm = confirm
        self.cancel = cancel

    def onClick(self, isChecked):
        if not isChecked:
            self.uncheck()
            return
        self.activate()

    def activate(self):
        self.confirm.activate(self.onConfirm)
        self.cancel.activate(self.onCancel)

    def deactivate(self):
        self.confirm.deactivate()
        self.cancel.deactivate()

    def onCancel(self):
        self.deactivate()
        self.setChecked(False)

    def onConfirm(self):
        self.deactivate()
        self.updateLinks()
        self.execute()


class ResumeButton(LinkableButton):
    def __init__(self, exec_id, radio_pub, radio900_pub):
        super(ResumeButton, self).__init__("Resume")
        self.exec_id = exec_id
        self.radio = radio_pub
        self.radio900 = radio900_pub
        self.setStyleSheet("QPushButton:checked {" + COLORS.GREEN + "}")

    def execute(self):
        radio900(self.radio900, NineHundredRadioMsg.ESTOP_RESUME, self.exec_id)
        radioStop(self.radio, RadioMsg.ESTOP_RESUME, self.exec_id)


class TakeoffButton(LinkableButton):
    def __init__(self, exec_id, radio_pub, radio900_pub):
        super(TakeoffButton, self).__init__("Resume/Takeoff")
        self.exec_id = exec_id
        self.radio = radio_pub
        self.radio900 = radio900_pub
        self.setStyleSheet("QPushButton:checked {" + COLORS.GREEN + "}")

    def execute(self):
        radio900(self.radio900, NineHundredRadioMsg.ESTOP_RESUME, self.exec_id)
        radioStop(self.radio, RadioMsg.ESTOP_RESUME, self.exec_id)


class SoftEStopButton(LinkableButton):
    def __init__(self, exec_id, radio_pub):
        super(SoftEStopButton, self).__init__("Soft E-Stop")
        self.exec_id = exec_id
        self.radio = radio_pub

    def execute(self):
        radioStop(self.radio, RadioMsg.ESTOP_SOFT, self.exec_id)

    # Allows programmatical way to activate this button from a mega E-Stop button.
    def forceStop(self):
        self.updateLinks()
        self.setChecked(True)
        self.execute()


class LandButton(LinkableConfirmButton):
    def __init__(self, confirm, cancel, exec_id, radio_pub):
        super(LandButton, self).__init__("Land", confirm, cancel)
        self.exec_id = exec_id
        self.radio = radio_pub

    def execute(self):
        radioStop(self.radio, RadioMsg.ESTOP_SOFT, self.exec_id)

    # Allows programmatical way to activate this button from a mega E-Stop button.
    def forceStop(self):
        self.updateLinks()
        self.setChecked(True)
        self.execute()


class HardEStopButton(LinkableConfirmButton):
    def __init__(self, confirm, cancel, exec_id, radio_pub, radio900_pub):
        super(HardEStopButton, self).__init__("Hard E-Stop", confirm, cancel)
        self.exec_id = exec_id
        self.radio = radio_pub
        self.radio900 = radio900_pub

    def execute(self):
        radio900(self.radio900, 1, self.exec_id)
        radioStop(self.radio, RadioMsg.ESTOP_HARD, self.exec_id)


class AerialHardEStopButton(LinkableConfirmButton):
    def __init__(self, confirm, cancel, exec_id, radio_pub, radio900_pub):
        super(AerialHardEStopButton, self).__init__("Hard E-Stop", confirm, cancel)
        self.exec_id = exec_id
        self.radio = radio_pub
        self.radio900 = radio900_pub
        # Not started but initialized to avoid checking for None in uncheck.
        self.timer = threading.Timer(1, self.execute)

    def uncheck(self):
        self.timer.cancel()

    def execute(self):
        self.timer.cancel()
        radio900(self.radio900, 1, self.exec_id)
        radioStop(self.radio, RadioMsg.ESTOP_HARD, self.exec_id)
        SECOND = 1
        self.timer = threading.Timer(2 * SECOND, self.execute)
        self.timer.start()


class HoverButton(LinkableButton):
    def __init__(self, exec_id, radio_pub):
        super(HoverButton, self).__init__("Hover")
        self.exec_id = exec_id
        self.radio = radio_pub

    def execute(self):
        radioStop(self.radio, RadioMsg.ESTOP_PAUSE, self.exec_id)


class ReturnHomeButton(LinkableButton):
    def __init__(self, exec_id, radio_pub):
        super(ReturnHomeButton, self).__init__("Return Home")
        self.exec_id = exec_id
        self.radio = radio_pub
        self.setStyleSheet("QPushButton:checked {" + COLORS.GREEN + "}")

    def execute(self):
        radio(self.radio, RadioMsg.MESSAGE_TYPE_RETURN_HOME, "", self.exec_id)


class ExploreButton(LinkableButton):
    def __init__(self, text, exec_id, radio_pub):
        super(ExploreButton, self).__init__(text)
        self.text = text
        self.exec_id = exec_id
        self.radio = radio_pub
        self.msg_type = RadioMsg.MESSAGE_TYPE_LANDING_BEHAVIOR

    def uncheck(self):
        self.setText(self.text)
        radio(self.radio, self.msg_type, RadioMsg.LAND_AT_HOME, self.exec_id)

    def execute(self):
        self.setText("Explore")
        radio(self.radio, self.msg_type, RadioMsg.LAND_IN_COMMS, self.exec_id)


class DefineWaypointButton(LinkableButton):
    """
    DefineWaypointButton, unlike the other button commands, needs help from its
    owner to actually move a waypoint in RViz and publish it when complete.
    This button takes callbacks, 'moveWaypoint' and 'publishWaypoint', to perform
    the actual heavy lifting of manipulating a waypoint.  This design decision
    was to avoid pulling in subcription callbacks that need to manipulate internal
    state of defining actual waypoint data for individual robots.

    The 'moveWaypoint' callback should return True if an error occurred.  This allows
    the button to display its error form to allow the user to investigate the cause.

    It is important that each DefineWaypointButton for every robot should be linked
    together incase the user begins to define a waypoint for one robot and then changes
    their mind and decides to define a waypoint for another.  Since only one marker can
    be displayed in RViz at a time we would like to reset every other waypoint button
    when this occurs.
    """

    def __init__(self, robot, moveWaypoint, publishWaypoint):
        super(DefineWaypointButton, self).__init__("Define Waypoint")
        self.robot = robot
        self.moveWaypoint = moveWaypoint
        self.publishWaypoint = publishWaypoint
        self.has_error = False

    def updateLink(self):
        self.setText("Define Waypoint")

    def uncheck(self):
        self.setText("Define Waypoint")
        if self.has_error == False:
            self.publishWaypoint(self.robot)

    def execute(self):
        self.setText("Move RViz Marker")
        self.setStyleSheet("QPushButton:checked {" + COLORS.BLUE + "}")
        self.has_error = self.moveWaypoint(self.robot)
        if self.has_error:
            self.setText("Waypoint Error")
            self.setStyleSheet("QPushButton:checked {" + COLORS.RED + "}")


class ShowBluetoothButton(qt.QPushButton):
    def __init__(self):
        super(ShowBluetoothButton, self).__init__("Show Bluetooth")
        self.setStyleSheet("QPushButton:pressed {" + COLORS.RED + "}")
        self.clicked.connect(self.onClick)

    def onClick(self):
        # The original implementation didn't do anything with this command.
        pass
