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
import re
import zlib

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
from std_msgs.msg import String, Float32
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

    def __init__(self, context):
        super(RobotCommandPlugin, self).__init__(context)
        self.setObjectName("RobotCommandPlugin")
        self.config = robots.Config()
        self.timer_buttons = []
        self.positions = dict()
        self.waypoints = dict()
        self.treeWidgets = dict()
        self.prevTrees = dict()
        self.waypt_robot = None
        self.subscriptions = []

        self.tree_trigger.connect(self.onTreeUpdateMonitor)

        def pub(to, what, size):
            return rospy.Publisher(to, what, queue_size=size)

        self.gui_message_pub = pub("/gui/message_print", GuiMessage, 10)
        self.waypoint_marker_on_pub = pub("/define_waypoint_marker_pos", Point, 50)
        self.waypoint_marker_off_pub = pub("/define_waypoint_marker_off", Point, 50)

        def radioize(robot):
            t = "/{0}/{1}".format(robot.topic_prefix, robot.topics["radio_command"])
            radio_cmd = pub(t, bsm.Radio, 50)
            send_count = [16]

            def send(msg_type, what):
                cmd = bsm.Radio()
                cmd.message_type = msg_type
                cmd.data = what
                for n in xrange(send_count[0]):
                    radio_cmd.publish(cmd)
                send_count[0] = 1

            def stop(what):
                send(bsm.Radio.MESSAGE_TYPE_ESTOP, what)

            t = "/{0}/{1}".format(robot.topic_prefix, robot.topics["clear_cloud"])
            cloud = pub(t, Float32, 10)
            clear_count = [16]

            def clear():
                # The number 42 was arbitrarily chosen by the team.
                msg = Float32()
                msg.data = 42.0
                for n in xrange(clear_count[0]):
                    cloud.publish(msg)
                clear_count[0] = 1

            robot.radio = send
            robot.radioStop = stop
            robot.clearCloud = clear

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
            radioize(r)
            self.treeWidgets[r.uuid] = xdot.DotWidget()
            self.prevTrees[r.uuid] = ""
            sub(odom, Odometry, self.onOdometry, r)
            sub(btree, String, self.onTreeUpdate, r)

        context.add_widget(self.createPanel())

    def shutdown_plugin(self):
        for s in self.subscriptions:
            s.unregister()
        for b in self.timer_buttons:
            b.uncheck(False)

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
            w.set_dotcode(zlib.decompress(msg.data))
            if pt == "":
                w.zoom_to_fit()
        self.prevTrees[robot.uuid] = msg.data

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

        def make_send_velocity(combo_box, robot):
            def fn():
                d = combo_box.currentText()
                robot.radio(bsm.Radio.MESSAGE_TYPE_MAX_FLIGHT_VELOCITY, d)

            return fn

        def make_send_comms(line_edit, robot):
            def fn():
                MB = qt.QMessageBox
                text = line_edit.text()
                nums = re.split(r'\D+', text)
                count = 0
                data = []
                if len(nums) < 2:
                    MB.critical(
                        None,
                        "Basestation",
                        "Must have at least two nodes for robot and basestation.",
                        buttons=MB.Ok,
                        defaultButton=MB.Ok,
                    )
                    return
                else:
                    count = len(nums) - 2

                for num in nums:
                    n = int(num)
                    if n == 0:
                        MB.critical(
                            None,
                            "Basestation",
                            "Node number can't be zero.",
                            buttons=MB.Ok,
                            defaultButton=MB.Ok,
                        )
                        return
                    else:
                        data.append("10.1.1.{0}".format(10 + n))

                answer = MB.question(
                    None,
                    "Basestation",
                    "Really update comms planner and behavior executive?  Ensure robot is not exploring.",
                    buttons=MB.Yes | MB.Cancel,
                    defaultButton=MB.Yes,
                )
                if answer == MB.Cancel:
                    return

                robot.radio(bsm.Radio.MESSAGE_TYPE_COMM_NODE_IP_LIST, str(data))
                robot.radio(bsm.Radio.MESSAGE_TYPE_COMM_NODE_COUNT, str(count))

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
            resume = ResumeButton(robot)
            joystick = JoystickButton(robot)
            home = ReturnHomeButton(robot)
            explore = ExploreButton(robot)
            soft = SoftEStopButton(robot)
            hard = HardEStopButton(robot)
            terrain = ResetTerrainAnalysisButton(robot)
            dropType = bsm.Radio.MESSAGE_TYPE_DROP_COMMS
            dropComms = RadioButton("Drop Comms", robot, dropType)
            send2dType = bsm.Radio.MESSAGE_TYPE_SEND_2D_MAP
            send3dType = bsm.Radio.MESSAGE_TYPE_SEND_3D_MAP
            send2d = RadioButton("Send 2D Map", robot, send2dType)
            send3d = RadioButton("Send 3D Map", robot, send3dType)
            waypt = DefineWaypointButton(
                robot, self.moveWaypoint, self.publishWaypoint, self.removeWaypoint
            )

            self.timer_buttons.extend([resume, home, soft, hard, joystick])

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
            cmd.leftButtons.addWidget(send2d)
            cmd.rightButtons.addWidget(waypt)
            cmd.rightButtons.addWidget(explore)
            cmd.rightButtons.addWidget(resend)
            cmd.rightButtons.addWidget(terrain)
            cmd.rightButtons.addWidget(dropComms)
            cmd.rightButtons.addWidget(send3d)

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
                hideButton(cmd.updateCommsButton)

            if robot.has_comms:
                box = qt.QLineEdit()
                cmd.extraInputLayout.addWidget(qt.QLabel("Nodes (Robot, BST, Nodes)"))
                cmd.extraInputLayout.addWidget(box)
                cmd.updateCommsButton.clicked.connect(make_send_comms(box, robot))

            cmd.leftButtons.addStretch()
            cmd.rightButtons.addStretch()
            cmd.eStopAllButton.clicked.connect(stopAll)
            cmd.treeLayout.addWidget(tree)
            tab.setLayout(grid)
            tabs.addTab(tab, robot.name)
            grid.addWidget(cmd)

            max_time = (2 * robot.max_travel_time) + 1
            times = [str(n / 2.0) for n in range(0, max_time)]
            box = cmd.travelTimeComboBox
            for t in times:
                box.addItem(t)
            box.currentTextChanged.connect(make_adjuster(box, robot))

            if robot.is_aerial:
                cmd.travelTimeLabel.setText("Max Travel Time (minutes)")
                box = qt.QComboBox()
                cmd.extraInputLayout.addWidget(qt.QLabel("Flight Speed (m/s)"))
                cmd.extraInputLayout.addWidget(box)
                speed = 0.1
                while speed < robot.max_flight_speed:
                    box.addItem("{0:.1f}".format(speed))
                    speed += 0.1
                box.currentTextChanged.connect(make_send_velocity(box, robot))

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
        self.timer = threading.Timer(1, self.execute)

    def execute(self):
        SECOND = 1
        self.timer.cancel()
        self.robot.radioStop(bsm.Radio.ESTOP_RESUME)
        self.timer = threading.Timer(1 * SECOND, self.execute)
        if not self.robot.is_aerial:
            self.timer.start()
        return True

    def uncheck(self, wasClicked):
        self.timer.cancel()


class JoystickButton(LinkableButton):
    def __init__(self, robot):
        super(JoystickButton, self).__init__("Joystick Control")
        self.robot = robot
        self.setStyleSheet("QPushButton:checked {" + COLORS.GREEN + "}")
        self.timer = threading.Timer(1, self.execute)

    def execute(self):
        SECOND = 1
        self.timer.cancel()
        self.robot.radioStop(bsm.Radio.ESTOP_JOYSTICK)
        self.timer = threading.Timer(1 * SECOND, self.execute)
        if not self.robot.is_aerial:
            self.timer.start()
        return True

    def uncheck(self, wasClicked):
        self.timer.cancel()


class SoftEStopButton(LinkableButton):
    def __init__(self, robot):
        self.text = "Soft E-Stop"
        if robot.is_aerial:
            self.text = "Land"
        super(SoftEStopButton, self).__init__(self.text)
        self.robot = robot
        self.timer = threading.Timer(1, self.repeatRadioStop)

    def execute(self, bypassConfirm=False):
        self.timer.cancel()

        if not self.robot.is_aerial:
            self.robot.radioStop(bsm.Radio.ESTOP_SOFT)
            self.startTimer()
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
        if not self.robot.is_aerial:
            self.startTimer()
        return True

    def uncheck(self, wasClicked):
        self.timer.cancel()

    def repeatRadioStop(self):
        self.timer.cancel()
        self.robot.radioStop(bsm.Radio.ESTOP_SOFT)
        self.startTimer()

    def startTimer(self):
        SECOND = 1
        self.timer = threading.Timer(1 * SECOND, self.repeatRadioStop)
        self.timer.start()

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
        self.timer = threading.Timer(1, self.repeatRadioStop)

    def uncheck(self, wasClicked):
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
            self.darpa_estop(self.robot.estop_disengage)
            self.was_darpa_estopped = False

    def execute(self):
        self.timer.cancel()

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
        if not self.robot.is_aerial:
            self.startTimer()
        return True

    def repeatRadioStop(self):
        self.timer.cancel()
        self.robot.radioStop(bsm.Radio.ESTOP_HARD)
        self.startTimer()

    def startTimer(self):
        SECOND = 1
        self.timer = threading.Timer(1 * SECOND, self.repeatRadioStop)
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
        self.timer = threading.Timer(1, self.execute)

    def execute(self):
        SECOND = 1
        self.timer.cancel()
        self.robot.radio(bsm.Radio.MESSAGE_TYPE_RETURN_HOME, "")
        self.timer = threading.Timer(1 * SECOND, self.execute)
        if not self.robot.is_aerial:
            self.timer.start()
        return True

    def uncheck(self, wasClicked):
        self.timer.cancel()


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


class ResetTerrainAnalysisButton(qt.QPushButton):
    def __init__(self, robot):
        super(ResetTerrainAnalysisButton, self).__init__("Reset Terrain Analysis")
        self.robot = robot
        self.setStyleSheet("QPushButton:clicked {" + COLORS.BLUE + "}")
        self.clicked.connect(self.execute)

    def execute(self):
        self.robot.clearCloud()
        return True
