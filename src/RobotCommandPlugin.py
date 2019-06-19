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
import serial

import python_qt_binding.QtWidgets as qt

from qt_gui.plugin import Plugin
from PyQt5.QtCore import pyqtSignal
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy, QTabWidget

import basestation_msgs.msg as bsm

from basestation_gui_python.msg import GuiMessage
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import InteractiveMarkerFeedback

from gui_utils import COLORS


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

        self.gui_message_pub = pub("/gui/message_print", GuiMessage, 10)
        self.waypoint_marker_on_pub = pub("/define_waypoint_marker_pos", Point, 50)
        self.waypoint_marker_off_pub = pub("/define_waypoint_marker_off", Point, 50)

        def radioize(robot):
            t = "/{0}/{1}".format(robot.topic_prefix, robot.topics["radio_command"])
            radio_cmd = pub(t, bsm.Radio, 50)

            def send(msg_type, what):
                cmd = bsm.Radio()
                cmd.message_type = msg_type
                cmd.data = what
                radio_cmd.publish(cmd)

            def stop(what):
                send(bsm.Radio.MESSAGE_TYPE_ESTOP, what)

            robot.radio = send
            robot.radioStop = stop

        self.subs = [
            rospy.Subscriber(
                "/define_waypoint/feedback", InteractiveMarkerFeedback, self.onWaypoint
            )
        ]

        for r in self.config.robots:
            t = "/{0}/{1}".format(r.topic_prefix, r.topics["odometry"])
            self.subs.append(rospy.Subscriber(t, Odometry, self.onOdometry, r))
            radioize(r)

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
            robot.radio(bsm.Radio.MESSAGE_TYPE_DEFINE_WAYPOINT, d)
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
        waypt_links = []
        stoppers = []

        label.setText("CONTROL PANEL")
        label.setAlignment(Qt.AlignCenter)
        layout.addWidget(label)
        layout.addWidget(tabs)
        widget.setLayout(layout)

        def make_adjuster(combo_box, robot):
            def fn():
                d = str(float(combo_box.currentText()) * 60)
                robot.radio(bsm.Radio.MESSAGE_TYPE_MAX_FLIGHT_TIME, d)

            return fn

        MAX_ROWS = 5
        for robot in self.config.robots:
            row, col = 0, 0
            grid = qt.QGridLayout()
            tab = QWidget()
            buttons = []
            confirm = ConfirmButton("Confirm", COLORS.GREEN)
            cancel = ConfirmButton("Cancel", COLORS.RED)
            waypt = DefineWaypointButton(robot, self.moveWaypoint, self.publishWaypoint)
            blue = ShowBluetoothButton()
            goHome = ReturnHomeButton(robot)
            resend = RadioButton(
                "Re-send Artifacts", robot, bsm.Radio.MESSAGE_TYPE_RESEND_ALL_ARTIFACTS
            )

            for w in waypt_links:
                w.link([waypt])
                waypt.link([w])
            waypt_links.append(waypt)

            tab.setLayout(grid)
            tabs.addTab(tab, robot.name)

            if robot.is_aerial:
                hover = HoverButton(robot)
                takeoff = TakeoffButton(robot)
                hard = AerialHardEStopButton(confirm, cancel, robot)
                land = LandButton(confirm, cancel, robot)
                toComms = ExploreButton("Land in Comms", robot)

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
                resume = ResumeButton(robot)
                soft = SoftEStopButton(robot)
                hard = HardEStopButton(confirm, cancel, robot)
                toComms = ExploreButton("Return to Comms", robot)

                stoppers.append(soft)
                resume.link([soft, hard, goHome])
                soft.link([resume, hard, goHome])
                hard.link([resume, soft, goHome])
                goHome.link([resume, soft, hard])

                commands.extend([resume, soft, hard, waypt, goHome])
                if robot.has_comms:
                    mt = bsm.Radio.MESSAGE_TYPE_DROP_COMMS
                    commands.append(RadioButton("Drop Comms", robot, mt))
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
            box.currentTextChanged.connect(make_adjuster(box, robot))
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
    RadioButton is a simple fire and forget button that sends a Radio message when
    clicked.
    """

    def __init__(self, text, robot, msg_type):
        super(RadioButton, self).__init__(text)

        def onClick():
            robot.radio(msg_type, "")

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
    def __init__(self, robot):
        super(ResumeButton, self).__init__("Resume")
        self.robot = robot
        self.setStyleSheet("QPushButton:checked {" + COLORS.GREEN + "}")

    def execute(self):
        self.robot.radioStop(bsm.Radio.ESTOP_RESUME)


class TakeoffButton(LinkableButton):
    def __init__(self, robot):
        super(TakeoffButton, self).__init__("Resume/Takeoff")
        self.robot = robot
        self.setStyleSheet("QPushButton:checked {" + COLORS.GREEN + "}")

    def execute(self):
        self.robot.radioStop(bsm.Radio.ESTOP_RESUME)


class SoftEStopButton(LinkableButton):
    def __init__(self, robot):
        super(SoftEStopButton, self).__init__("Soft E-Stop")
        self.robot = robot

    def execute(self):
        self.robot.radioStop(bsm.Radio.ESTOP_SOFT)

    # Allows programmatical way to activate this button from a mega E-Stop button.
    def forceStop(self):
        self.updateLinks()
        self.setChecked(True)
        self.execute()


class LandButton(LinkableConfirmButton):
    def __init__(self, confirm, cancel, robot):
        super(LandButton, self).__init__("Land", confirm, cancel)
        self.robot = robot

    def execute(self):
        self.robot.radioStop(bsm.Radio.ESTOP_SOFT)

    # Allows programmatical way to activate this button from a mega E-Stop button.
    def forceStop(self):
        self.updateLinks()
        self.setChecked(True)
        self.execute()


def darpa_estop(where, what):
    try:
        with serial.Serial(where) as s:
            s.write(what)
    except Exception as e:
        rospy.logerr("[Robot Command] Hard E-Stop: %s", e)


class HardEStopButton(LinkableConfirmButton):
    def __init__(self, confirm, cancel, robot):
        super(HardEStopButton, self).__init__("Hard E-Stop", confirm, cancel)
        self.robot = robot

    def uncheck(self):
        darpa_estop(self.robot.estop_serial_port, self.robot.estop_disengage)

    def execute(self):
        self.robot.radioStop(bsm.Radio.ESTOP_HARD)
        darpa_estop(self.robot.estop_serial_port, self.robot.estop_engage)


class AerialHardEStopButton(LinkableConfirmButton):
    def __init__(self, confirm, cancel, robot):
        super(AerialHardEStopButton, self).__init__("Hard E-Stop", confirm, cancel)
        self.robot = robot
        self.was_darpa_estopped = False
        # Not started but initialized to avoid checking for None in uncheck.
        self.timer = threading.Timer(1, self.execute)

    def uncheck(self):
        self.timer.cancel()
        # Due to the timer, the plugin has to forcefully uncheck this button, regardless
        # if it was previously checked, in order to properly shutdown the timer.
        # However, this can inadvertently send a DARPA disengage command which may not
        # be desired.  Imagine the sequence:
        #
        # 1) DARPA hard E-stops the drone with their own interface.
        # 2) GUI is subsequently shutdown for whatever reason.
        # 3) GUI disengages DARPA hard e-stop; thus, re-activating the robot.
        # 4) CMU is disqualified from the competition.
        #
        # Hence, why the 'was_darpa_estopped' flag is used to avoid this behavior.
        if self.was_darpa_estopped:
            darpa_estop(self.estop_serial_port, self.estop_disengage)
            self.was_darpa_estopped = False

    def execute(self):
        self.timer.cancel()
        self.robot.radioStop(bsm.Radio.ESTOP_HARD)
        darpa_estop(self.robot.estop_serial_port, self.robot.estop_engage)
        self.was_darpa_estopped = True
        SECOND = 1
        self.timer = threading.Timer(2 * SECOND, self.execute)
        self.timer.start()


class HoverButton(LinkableButton):
    def __init__(self, robot):
        super(HoverButton, self).__init__("Hover")
        self.robot = robot

    def execute(self):
        self.robot.radioStop(bsm.Radio.ESTOP_PAUSE)


class ReturnHomeButton(LinkableButton):
    def __init__(self, robot):
        super(ReturnHomeButton, self).__init__("Return Home")
        self.robot = robot
        self.setStyleSheet("QPushButton:checked {" + COLORS.GREEN + "}")

    def execute(self):
        self.robot.radio(bsm.Radio.MESSAGE_TYPE_RETURN_HOME, "")


class ExploreButton(LinkableButton):
    def __init__(self, text, robot):
        super(ExploreButton, self).__init__(text)
        self.text = text
        self.robot = robot
        self.msg_type = bsm.Radio.MESSAGE_TYPE_LANDING_BEHAVIOR

    def uncheck(self):
        self.setText(self.text)
        self.robot.radio(self.msg_type, bsm.Radio.LAND_AT_HOME)

    def execute(self):
        self.setText("Explore")
        self.robot.radio(self.msg_type, bsm.Radio.LAND_IN_COMMS)


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
