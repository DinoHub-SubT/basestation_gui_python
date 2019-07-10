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
import os

import xdot.xdot_qt as xdot
import python_qt_binding.QtWidgets as qt
import python_qt_binding.QtGui as gui

from qt_gui.plugin import Plugin
from PyQt5.QtCore import pyqtSignal
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy, QTabWidget

import basestation_msgs.msg as bsm

from basestation_gui_python.msg import GuiMessage
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import InteractiveMarkerFeedback

from gui_utils import COLORS


################################################################################
#
#    Plugin for building tabbed panel containing buttons to command the
#    various robots.
#
################################################################################


class RobotCommandPlugin(Plugin):
    tree_trigger = pyqtSignal(object, object)
    status_trigger = pyqtSignal(object, object)

    def __init__(self, context):
        super(RobotCommandPlugin, self).__init__(context)
        self.setObjectName("RobotCommandPlugin")
        self.config = robots.Config()
        self.timer_buttons = []
        self.positions = dict()
        self.waypoints = dict()
        self.treeWidgets = dict()
        self.prevTrees = dict()
        self.batteryStatus = dict()
        self.commsStatus = dict()
        self.cpuStatus = dict()
        self.diskSpaceStatus = dict()
        self.rssiStatus = dict()
        self.waypt_robot = None
        self.subscriptions = []

        self.tree_trigger.connect(self.onTreeUpdateMonitor)
        self.status_trigger.connect(self.onStatusUpdateMonitor)

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

        def sub(topic, what, callback, extra=None):
            if extra == None:
                s = rospy.Subscriber(topic, what, callback)
            else:
                s = rospy.Subscriber(topic, what, callback, extra)
            self.subscriptions.append(s)

        sub("/define_waypoint/feedback", InteractiveMarkerFeedback, self.onWaypoint)

        for r in self.config.robots:
            odom = "/{0}/{1}".format(r.topic_prefix, r.topics["odometry"])
            btree = "/{0}/{1}".format(r.topic_prefix, r.topics["behavior_tree"])
            status = "/{0}/{1}".format(r.topic_prefix, r.topics["status_update"])
            radioize(r)
            self.treeWidgets[r.uuid] = xdot.DotWidget()
            self.prevTrees[r.uuid] = ""
            sub(odom, Odometry, self.onOdometry, r)
            sub(btree, String, self.onTreeUpdate, r)
            sub(status, bsm.StatusUpdate, self.onStatusUpdate, r)

        context.add_widget(self.createPanel())

    def shutdown_plugin(self):
        for s in self.subscriptions:
            s.unregister()
        for b in self.timer_buttons:
            b.uncheck()

    def removeWaypoint(self):
        self.waypoint_marker_off_pub.publish(Point(-1, -1, -1))

    def publishWaypoint(self, robot):
        """Callback to publish a waypoint for the robot to move to."""
        if self.waypoints.has_key(robot.uuid):
            w = self.waypoints[robot.uuid]
            d = "{0},{1},{2}".format(w[0], w[1], w[2])
            robot.radio(bsm.Radio.MESSAGE_TYPE_DEFINE_WAYPOINT, d)
            self.waypt_robot = None
            return ""
        else:
            m = GuiMessage()
            m.data = "No waypoint received from RViz."
            m.color = m.COLOR_GRAY
            self.gui_message_pub.publish(m)
            return m.data

    def moveWaypoint(self, robot):
        """Callback to have RViz place an interactive marker to define a waypoint."""
        if self.positions.has_key(robot.uuid):
            p = self.positions.get(robot.uuid)
            self.waypt_robot = robot
            self.waypoint_marker_on_pub.publish(Point(p[0], p[1], p[2]))
            return ""
        else:
            text = "No odometry received for {0}.  Check topic from setup file."
            m = GuiMessage()
            m.data = text.format(robot.name)
            m.color = m.COLOR_GRAY
            self.gui_message_pub.publish(m)
            return m.data

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

    def onTreeUpdate(self, msg, robot):
        self.tree_trigger.emit(msg, robot)

    def onTreeUpdateMonitor(self, msg, robot):
        pt = self.prevTrees[robot.uuid]
        if msg.data != pt:
            w = self.treeWidgets[robot.uuid]
            w.set_dotcode(msg.data)
            if pt == "":
                w.zoom_to_fit()
        self.prevTrees[robot.uuid] = msg.data

    def onStatusUpdate(self, msg, robot):
        self.status_trigger.emit(msg, robot)

    def onStatusUpdateMonitor(self, msg, robot):
        SU = bsm.StatusUpdate
        status = None

        if msg.what == SU.WHAT_BATTERY:
            status = self.batteryStatus[robot.uuid]
        elif msg.what == SU.WHAT_COMMS:
            status = self.commsStatus[robot.uuid]
        elif msg.what == SU.WHAT_CPU:
            status = self.cpuStatus[robot.uuid]
        elif msg.what == SU.WHAT_DISK_SPACE:
            status = self.diskSpaceStatus[robot.uuid]
        elif msg.what == SU.WHAT_RSSI:
            status = self.rssiStatus[robot.uuid]
        elif msg.what == SU.WHAT_MOBILITY:
            # Mobility provides redundant information to the behavior tree
            # so it is removed to save GUI space.
            return

        status.setText(msg.value)
        if msg.severity == SU.SEVERITY_CRITICAL:
            status.setStyleSheet(COLORS.LIGHT_RED)
        elif msg.severity == SU.SEVERITY_WARNING:
            status.setStyleSheet(COLORS.YELLOW)
        else:
            status.setStyleSheet(None)

    def createPanel(self):
        widget = QWidget()
        layout = qt.QGridLayout()
        tabs = QTabWidget()
        stoppers = []

        layout.addWidget(tabs)
        widget.setWindowTitle("Robot Control")
        widget.setLayout(layout)

        def make_adjuster(combo_box, robot):
            def fn():
                d = str(float(combo_box.currentText()) * 60)
                robot.radio(bsm.Radio.MESSAGE_TYPE_MAX_FLIGHT_TIME, d)

            return fn

        def stopAll():
            MB = qt.QMessageBox
            answer = MB.question(
                None,
                "Basestation",
                "Really Soft E-Stop all robots?",
                buttons=MB.Yes | MB.Cancel,
                defaultButton=MB.Yes,
            )
            if answer == MB.Cancel:
                return
            for s in stoppers:
                s.forceStop()

        def hideButton(btn):
            sp = btn.sizePolicy()
            sp.setRetainSizeWhenHidden(True)
            btn.setSizePolicy(sp)
            btn.setVisible(False)

        for robot in self.config.robots:
            grid = qt.QGridLayout()
            tab = QWidget()
            cmd = CommandWidget(robot.is_aerial)
            tree = self.treeWidgets[robot.uuid]
            resendType = bsm.Radio.MESSAGE_TYPE_RESEND_ALL_ARTIFACTS
            resend = RadioButton("Resend Artifacts", robot, resendType)
            hidden = RadioButton("Hidden", robot, resendType)
            resume = ResumeButton(robot)
            joystick = JoystickButton(robot)
            home = ReturnHomeButton(robot)
            explore = ExploreButton(robot)
            soft = SoftEStopButton(robot)
            hard = HardEStopButton(robot)
            waypt = DefineWaypointButton(
                robot, self.moveWaypoint, self.publishWaypoint, self.removeWaypoint
            )

            hideButton(hidden)

            self.batteryStatus[robot.uuid] = cmd.batteryLabel
            self.commsStatus[robot.uuid] = cmd.commsLabel
            self.cpuStatus[robot.uuid] = cmd.cpuLabel
            self.diskSpaceStatus[robot.uuid] = cmd.diskSpaceLabel
            self.rssiStatus[robot.uuid] = cmd.rssiLabel

            stoppers.append(soft)
            resume.link([soft, hard, home, explore, joystick])
            home.link([resume, soft, hard, explore, joystick])
            soft.link([hard, resume, home, explore, joystick])
            hard.link([resume, soft, home, explore, joystick])
            explore.link([resume, soft, hard, home, joystick])
            joystick.link([resume, soft, hard, home, explore])

            cmd.leftButtons.addWidget(resume)
            cmd.leftButtons.addWidget(home)
            cmd.leftButtons.addWidget(soft)
            cmd.leftButtons.addWidget(hard)
            cmd.leftButtons.addWidget(joystick)
            cmd.rightButtons.addWidget(waypt)
            cmd.rightButtons.addWidget(explore)
            cmd.rightButtons.addWidget(resend)

            if robot.is_aerial:
                hover = HoverButton(robot)
                hover.link([resume, soft, hard, home, explore, joystick])
                resume.link([hover])
                home.link([hover])
                soft.link([hover])
                hard.link([hover])
                explore.link([hover])
                joystick.link([hover])
                cmd.rightButtons.addWidget(hover)
            else:
                mt = bsm.Radio.MESSAGE_TYPE_DROP_COMMS
                btn = RadioButton("Drop Comms", robot, mt)
                cmd.rightButtons.addWidget(btn)
                # If the ground robot doesn't have a comm dropper then
                # we'll hide the button but first the size policy must
                # be retained; otherwise, the other buttons will fill
                # the space of the hidden button.
                if not robot.has_comms:
                    hideButton(btn)

            cmd.rightButtons.addWidget(hidden)
            cmd.eStopAllButton.clicked.connect(stopAll)
            cmd.treeLayout.addWidget(tree)
            tab.setLayout(grid)
            tabs.addTab(tab, robot.name)
            grid.addWidget(cmd)

            max_time = (2 * robot.max_travel_time) + 1
            times = [str(n / 2.0) for n in range(0, max_time)]
            box = cmd.travelTimeComboBox
            for speed in times:
                box.addItem(speed)
            box.currentTextChanged.connect(make_adjuster(box, robot))

        return widget


class CommandWidget(qt.QWidget):
    def __init__(self, isAerial):
        super(CommandWidget, self).__init__()
        self.setObjectName("CommandWidget")
        bs = rospkg.RosPack().get_path("basestation_gui_python")
        main = os.path.join(bs, "resources", "robot_command.ui")
        loadUi(main, self, {})


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
            self.uncheck(True)
            return
        executed = self.execute()
        if executed:
            self.updateLinks()
        else:
            self.setChecked(False)

    def updateLinks(self):
        for btn in self.linked:
            btn.uncheck(False)
            btn.setChecked(False)

    def link(self, buttons):
        """
        Link other checkable buttons that will be un-checked when this button is
        activated.
        """
        self.linked.extend(buttons)

    def uncheck(self, wasClicked):
        """
        Allow derived classes to execute custom logic when the button is unchecked.

        WasClicked indicates if this call was due to the button being physically
        clicked.
        """
        pass

    def execute(self):
        """
        Execute needs to be implemented by derived classes to execute custom behavior
        for when the button is checked.  Should return true if the action was
        successful/confirmed.
        """
        raise NotImplementedError()


class ResumeButton(LinkableButton):
    def __init__(self, robot):
        text = "Resume"
        if robot.is_aerial:
            text = "Resume/Takeoff"
        super(ResumeButton, self).__init__(text)
        self.robot = robot
        self.setStyleSheet("QPushButton:checked {" + COLORS.GREEN + "}")

    def execute(self):
        self.robot.radioStop(bsm.Radio.ESTOP_RESUME)
        return True


class JoystickButton(LinkableButton):
    def __init__(self, robot):
        super(JoystickButton, self).__init__("Joystick Control")
        self.robot = robot
        self.setStyleSheet("QPushButton:checked {" + COLORS.GREEN + "}")

    def execute(self):
        self.robot.radioStop(bsm.Radio.ESTOP_JOYSTICK)
        return True


class SoftEStopButton(LinkableButton):
    def __init__(self, robot):
        self.text = "Soft E-Stop"
        if robot.is_aerial:
            self.text = "Land"
        super(SoftEStopButton, self).__init__(self.text)
        self.robot = robot

    def execute(self, bypassConfirm=False):
        if not self.robot.is_aerial:
            self.robot.radioStop(bsm.Radio.ESTOP_SOFT)
            return True

        if not bypassConfirm:
            MB = qt.QMessageBox
            answer = MB.question(
                None,
                "Basestation",
                "Really {0} the robot?".format(self.text),
                buttons=MB.Yes | MB.Cancel,
                defaultButton=MB.Yes,
            )
            if answer == MB.Cancel:
                return False

        self.robot.radioStop(bsm.Radio.ESTOP_SOFT)
        return True

    # Allows programmatical way to activate this button from a mega E-Stop button.
    def forceStop(self):
        self.updateLinks()
        self.setChecked(True)
        self.execute(bypassConfirm=True)


class HardEStopButton(LinkableButton):
    def __init__(self, robot):
        super(HardEStopButton, self).__init__("Hard E-Stop")
        self.robot = robot
        self.was_darpa_estopped = False
        # Not started but initialized to avoid checking for None in uncheck.
        if self.robot.is_aerial:
            self.timer = threading.Timer(1, self.execute)

    def uncheck(self, wasClicked):
        self.cancelTimer()
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
            self.darpa_estop(self.robot.estop_disengage)
            self.was_darpa_estopped = False

    def execute(self):
        self.cancelTimer()

        MB = qt.QMessageBox
        answer = MB.question(
            None,
            "Basestation",
            "Really Hard E-Stop the robot?",
            buttons=MB.Yes | MB.Cancel,
            defaultButton=MB.Yes,
        )
        if answer == MB.Cancel:
            return False

        self.robot.radioStop(bsm.Radio.ESTOP_HARD)
        self.was_darpa_estopped = self.darpa_estop(self.robot.estop_engage)
        self.startTimer()
        return True

    def repeatRadioStop(self):
        self.cancelTimer()
        self.robot.radioStop(bsm.Radio.ESTOP_HARD)
        self.startTimer()

    def cancelTimer(self):
        if self.robot.is_aerial:
            self.timer.cancel()

    def startTimer(self):
        if not self.robot.is_aerial:
            return
        SECOND = 1
        self.timer = threading.Timer(2 * SECOND, self.repeatRadioStop)
        self.timer.start()

    def darpa_estop(self, what):
        try:
            with serial.Serial(self.robot.estop_serial_port) as s:
                s.write(what)
                return True
        except Exception as e:
            MB = qt.QMessageBox
            MB.critical(
                None,
                "Basestation",
                "DARPA Hard E-Stop error: {0}: ".format(e),
                buttons=MB.Ok,
                defaultButton=MB.Ok,
            )
            return False


class HoverButton(LinkableButton):
    def __init__(self, robot):
        super(HoverButton, self).__init__("Hover")
        self.robot = robot

    def execute(self):
        self.robot.radioStop(bsm.Radio.ESTOP_PAUSE)
        return True


class ReturnHomeButton(LinkableButton):
    def __init__(self, robot):
        super(ReturnHomeButton, self).__init__("Return Home")
        self.robot = robot
        self.setStyleSheet("QPushButton:checked {" + COLORS.GREEN + "}")

    def execute(self):
        self.robot.radio(bsm.Radio.MESSAGE_TYPE_RETURN_HOME, "")
        return True


class ExploreButton(LinkableButton):
    def __init__(self, robot):
        text = "Return to Comms"
        if robot.is_aerial:
            text = "Land In Comms"
        super(ExploreButton, self).__init__(text)
        self.text = text
        self.robot = robot
        self.msg_type = bsm.Radio.MESSAGE_TYPE_LANDING_BEHAVIOR
        self.setStyleSheet("QPushButton:checked {" + COLORS.BLUE + "}")

    def uncheck(self, wasClicked):
        if wasClicked:
            self.robot.radio(self.msg_type, bsm.Radio.LAND_AT_HOME)

    def execute(self):
        self.robot.radio(self.msg_type, bsm.Radio.LAND_IN_COMMS)
        return True


class DefineWaypointButton(qt.QPushButton):
    """
    DefineWaypointButton, unlike the other button commands, needs help from its owner to
    actually move a waypoint in RViz and publish it when complete.  This button takes
    callbacks, 'moveWaypoint', 'publishWaypoint', and 'removeWaypoint', to perform the
    actual heavy lifting of manipulating a waypoint.  This design decision was to avoid
    pulling in subcription callbacks that need to manipulate internal state of defining
    actual waypoint data for individual robots.

    The 'moveWaypoint' callback should return a message if an error occurred.  This
    allows the button to display the error to allow the user to investigate the cause.

    The 'removeWaypoint' is called whenever there is an error or when the user is done
    manipulating the waypoint.  This callback should have RViz remove the waypoint from
    its display.
    """

    def __init__(self, robot, moveWaypoint, publishWaypoint, removeWaypoint):
        super(DefineWaypointButton, self).__init__("Define Waypoint")
        self.robot = robot
        self.moveWaypoint = moveWaypoint
        self.publishWaypoint = publishWaypoint
        self.removeWaypoint = removeWaypoint
        self.clicked.connect(self.execute)

    def execute(self):
        MB = qt.QMessageBox
        who = "Basestation"
        error = self.moveWaypoint(self.robot)
        if error != "":
            self.removeWaypoint()
            MB.critical(None, who, error, buttons=MB.Ok, defaultButton=MB.Ok)
            return

        what = "Move marker in RViz then click 'Apply'."
        btns = MB.Cancel | MB.Apply
        answer = MB.information(None, who, what, buttons=btns, defaultButton=MB.Apply)
        if answer == MB.Cancel:
            self.removeWaypoint()
            return
        error = self.publishWaypoint(self.robot)
        if error != "":
            MB.warning(None, who, error, buttons=MB.Ok, defaultButton=MB.Ok)
        self.removeWaypoint()
