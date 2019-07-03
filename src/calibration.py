from __future__ import print_function

import os
import math
import random
import datetime
import json
import re
import subprocess
import rospkg
import rospy
import robots

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore, QtGui, QtWidgets
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point


class Robot(object):
    """
    Robot provides a structure for the UI to communicate with its controller and has the
    following fields:

    - name
    Copied from a robots.Robot.

    - is_aerial
    True if the robot is an aerial (drone) robot.  Copied from a robots.Robot.

    - points
    Number of points saved so far.

    - last_save
    Records the datetime of when either the last successful point save operation has
    been performed on the robot in the UI.  This should be reset to the empty string
    if the user resets the points.

    - uuid
    A random unique identifier that conforms to the UUID4 specification.  This created
    once and should never be changed.  Copied from a robots.Robot.

    - mean_error
    The computed mean error after calibrating the transform matrix.

    - transform:
    A 4x4 matrix that can transform CMU's Subt coordinate frame to DARPA's coordinate
    frame.  The default is the identity matrix.

    - quaternion:
    The rotation portion of 'transform' represented in quaternion form.

    - darpa_tf_pub:
    ROS publisher of where to publish an Odometry message to the robot to utilize
    the calibrated DARPA transform.
    """

    def __init__(self, cfgRobot):
        self.name = cfgRobot.name
        self.is_aerial = cfgRobot.is_aerial
        self.points = []
        self.last_save = ""
        self.uuid = cfgRobot.uuid
        self.mean_error = 0
        self.transform = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        self.quaternion = [0, 0, 0, 1]
        self.darpa_tf_pub = None

    def encode_to_json(self):
        """Encode this Robot object into a dictionary that is suitable for JSON encoding."""
        d = dict()
        d["name"] = self.name
        d["uuid"] = self.uuid
        d["is_aerial"] = self.is_aerial
        d["points"] = self.points
        d["last_save"] = self.last_save
        d["mean_error"] = self.mean_error
        d["transform"] = self.transform
        d["quaternion"] = self.quaternion
        return d

    def decode_from_json(self, obj):
        """
        Decode a dictionary object that was loaded from JSON and return a new Robot object.

        Note that while all Robot properties are serialized to the JSON file they are not
        deserialized back into Robot as these properties are already derived from the
        configuration robots.Robot.
        """
        self.points = obj["points"]
        self.last_save = obj["last_save"]
        self.mean_error = obj["mean_error"]
        self.transform = obj["transform"]
        self.quaternion = obj["quaternion"]


class Calibration(Plugin):
    """
    Main entry point for the robot calibration plugin and acts as the controller
    for the CalibrationView.

    This plugin takes an overall different strategy then the old calibration workflow.
    Previously the GUI could only support two robots and the user had to run the
    calibration process in a separate terminal window.  This is no longer necessary
    as the this plugin performs such process on the users behalf while allowing an
    arbitrary number of robots.

    The support for any number of robots required a different strategy to interface
    with calibration process.  Now the two have been decoupled as this plugin maintains
    its own set of data files under the basestation_gui_python/data/calibration path.
    Inside this folder there will be a JSON file for each robot that is added through
    the user interface.  The name of the file is the UUID that was assigned to the
    Robot object when it was created.  The contents of this file is the Robot object
    serialized into JSON form.  Ideally, a database would be the right answer here
    but adding such a dependency is overkill what is needed from the GUI.

    With robots continously persisted into JSON files this also decouples this plugin
    from any other plugin that needs the resulting data.  For example, when a robot
    has detected an artifact, the other plugin that is managing artifact images can
    load the robot data to obtain its DARPA transformation matrix to apply to the
    artifacts coordinate frame.  In other words, any type of message passing between
    plugins is unnecessary.
    """

    def __init__(self, context):
        super(Calibration, self).__init__(context)
        self.setObjectName("Calibration")
        self.last_pose_total = None
        self.last_pose = dict()

        def odometry_to_pose(odometry):
            return [
                odometry.pose.pose.position.x,
                odometry.pose.pose.position.y,
                odometry.pose.pose.position.z,
            ]

        # Subscribe callbacks
        def make_pose_callback(robot):
            def on_pose(msg):
                self.last_pose[robot.uuid] = odometry_to_pose(msg)

            return on_pose

        def on_pose_total(msg):
            self.last_pose_total = odometry_to_pose(msg)

        self.subs = [rospy.Subscriber("/position", Odometry, on_pose_total)]

        cal_robots = []
        root = self.robot_dir()
        config = robots.Config()
        for cfg in config.robots:
            robot = Robot(cfg)
            name = robot.uuid + ".json"
            path = os.path.join(root, name)
            if os.path.exists(path):
                with open(path) as f:
                    robot.decode_from_json(json.load(f))
            else:
                with open(path, "w") as f:
                    json.dump(robot.encode_to_json(), f)

            topic = "/{0}/{1}".format(cfg.topic_prefix, cfg.topics["darpa_tf"])
            robot.darpa_tf_pub = rospy.Publisher(topic, Odometry, queue_size=10)

            topic = "/{0}/{1}".format(cfg.topic_prefix, cfg.topics["calibration"])
            sub = rospy.Subscriber(topic, Odometry, make_pose_callback(robot))
            self.subs.append(sub)
            cal_robots.append(robot)

        self.view = CalibrationView(self, cal_robots)
        context.add_widget(self.view)

    def shutdown_plugin(self):
        for s in self.subs:
            s.unregister()

    #################### private methods ####################
    def add_pose(self, robot, pose, total):
        robot.points.append([pose, total])
        robot.last_save = str(datetime.datetime.now())
        self.persist(robot)
        return robot

    def robot_dir(self):
        """Robot_dir returns the directory where Robot objects are archived."""
        p = rospkg.RosPack().get_path("basestation_gui_python")
        return os.path.join(p, "data", "calibration")

    def robot_filename(self, robot):
        """Robot_filename returns the full file path of where a Robot object should be archived."""
        return os.path.join(self.robot_dir(), robot.uuid + ".json")

    def persist(self, robot):
        """Persist archives _robot_ as json to the path specificed by robot_filename."""
        fn = self.robot_filename(robot)
        with open(fn, "w") as f:
            json.dump(robot.encode_to_json(), f)

    #################### CalibrationView interface methods ####################
    def name_changed(self, robot):
        self.persist(robot)

    def on_save(self, robot):
        have_pose = self.last_pose.has_key(robot.uuid)
        have_total = self.last_pose_total != None
        if have_pose and have_total:
            p = self.last_pose.get(robot.uuid)
            return self.add_pose(robot, p, self.last_pose_total)
        elif have_pose:
            return "Have not received total pose.  No point saved."
        else:
            return "Have not received robot pose.  No point saved."

    def on_reset(self, robot):
        robot.points = []
        robot.last_save = ""
        self.persist(robot)
        return robot

    def on_calibrate(self, robot):
        RPKG = rospkg.RosPack()
        ECAL = "entrance_calib"
        if ECAL not in RPKG.list():
            m = "The ROS '{0}' package was not found in the package list.  Ensure that it is installed and sourced."
            return (True, m.format(ECAL))
        if len(robot.points) < 2:
            # This error was from reading the calibration code and we perform it here in
            # order to get a fast and clear error message to the user.  The drawback is
            # that if someone modifies the calibration process and changes the number of
            # points required for a calibration then this has to change as well.  C'est la vie.
            return (
                True,
                "Not enough points.  Need 2 or more points for a calibration.",
            )

        # In a previous implementation the GUI program would record the data into the
        # calibration's data folder on the go while keeping separate files for the ground and
        # aerial vehicle.  Here we choose a different strategy where we already have the
        # necessary points cached in memory and persisted in our JSON files so we just
        # dump the output to the same location where the calibration process expects to
        # find the data.  Note that we always dump to the exact output location regardless
        # if the robot is a UGV or UAV as the calibration makes no differentiation in the
        # matter.  This keeps us from having to modify ROS parameters to tell the calibration
        # which files to load and just its defaults listed in its launch file.
        cal_data = os.path.join(RPKG.get_path("entrance_calib"), "data")
        est_path = os.path.join(cal_data, "data_points.txt")
        tot_path = os.path.join(cal_data, "reference_points.txt")
        with open(est_path, "w") as est:
            with open(tot_path, "w") as tot:
                this = "{0}\n".format(len(robot.points))
                est.write(this)
                tot.write(this)
                # Example of what robot.points looks like:
                #    [
                #      [ [1,2,3], [4,5,6] ],
                #      ...
                #      [ [1,2,3], [4,5,6] ],
                #    ]
                for e, t in robot.points:
                    e_pt = "{0} {1} {2}\n".format(e[0], e[1], e[2])
                    t_pt = "{0} {1} {2}\n".format(t[0], t[1], t[2])
                    est.write(e_pt)
                    tot.write(t_pt)

        # Call the actual calibration process.  Previously this step had to be done manually
        # as the user had to use both the terminal and GUI in conjuction with each other
        # to perform the calibration and upload the transformed frame to DARPA.
        out = ""
        try:
            out = subprocess.check_output(
                ["roslaunch", ECAL, ECAL + ".launch"], stderr=subprocess.STDOUT
            )
        except subprocess.CalledProcessError as e:
            bad = str(e) + ":\n\n" + out
            rospy.logerr(bad)
            return (True, bad)
        # This check is for the case when subprocess.CalledProcessError is not thrown.
        # A test was made to have 'entrance_calib' exit with a code of one by having one
        # of the files it was looking for not exist.  When doing so the exception was
        # thrown and it apppeared that ROS was 'swallowing' the exit code itself and
        # returning a new code of zero.
        if "exit code 1" in out:
            rospy.logerr(out)
            return (True, "'entrance_calib' has shutdown unexpectedly:\n\n" + out)

        # At this point we have successfully run the calibration and are now able
        # to proceed on transforming its results and save the DARPA transform.

        # Extract the mean error from the process standard output.
        match = re.search(
            "Mean error:\s*([.\w]+)", out, flags=re.MULTILINE | re.IGNORECASE
        )
        if not match:
            return (True, "No mean error detected in calibration output.")
        mean_error = float(match.group(1))  # save for later in case of other errors

        cal_path = os.path.join(cal_data, "calib.txt")
        # Example of what's in calib.txt:
        #    1 2 3
        #    4 5 6
        #    7 8 9
        #
        #    10
        #    11
        #    12
        #
        # The first three rows (items 1 thru 9) represent a 3x3 rotation matrix
        # while elements 10, 11, and 12 represents the translation vector.
        lines = []
        with open(cal_path) as f:
            lines = f.readlines()
        if len(lines) < 7:
            return (
                True,
                "Calibration output is missing necessary matrix data.  Transform unchanged.",
            )

        lines = [line.strip() for line in lines]  # strip newlines
        # Note that lines[3] is the empty string.
        r0 = map(float, lines[0].split(" "))
        r1 = map(float, lines[1].split(" "))
        r2 = map(float, lines[2].split(" "))
        r0.append(float(lines[4]))
        r1.append(float(lines[5]))
        r2.append(float(lines[6]))
        if len(r0) < 4 and len(r1) < 4 and len(r2) < 4:
            return (
                True,
                "Calibration output is missing necessary data.  Transform unchanged.",
            )

        robot.transform[0] = r0
        robot.transform[1] = r1
        robot.transform[2] = r2
        robot.mean_error = mean_error

        # The robot accepts transform as a combination of the translation
        # vector and a quaternion for the rotation.
        pt = Point(r0[3], r1[3], r2[3])
        qw = math.sqrt(1.0 + r0[0] + r1[1] + r2[2]) / 2.0
        qx = (r2[1] - r1[2]) / (4.0 * qw)
        qy = (r0[2] - r2[0]) / (4.0 * qw)
        qz = (r1[0] - r0[1]) / (4.0 * qw)
        qt = Quaternion(qx, qy, qz, qw)
        tf = Odometry()

        robot.quaternion = [qx, qy, qz, qw]
        tf.pose.pose.position = pt
        tf.pose.pose.orientation = qt

        self.persist(robot)
        robot.darpa_tf_pub.publish(tf)

        return (False, robot)


# The following constants represent the columns in the table of the calibration UI.
NAME_COL = 0
AERIAL_COL = 1
POINT_COL = 2
SAVE_COL = 3
RESET_COL = 4
TRANS_COL = 5
TIME_COL = 6


def confirm(title, message):
    """
    Presents a simple message box to the asking to confirm the operation.

    Returns QtWidgets.QMessageBox.  Yes if the user confirms; otherwise, returns
    QtWidgest.QMessageBox.Cancel.
    """
    MB = QtWidgets.QMessageBox
    return MB.warning(
        None, title, message, buttons=MB.Yes | MB.Cancel, defaultButton=MB.Yes
    )


class CalibrationView(QtWidgets.QWidget):
    """
    Everything UI to calibrate a robot and transforms its coordinate frame to
    be uploaded to DARPA.  The controller, the second argument to the constructor
    is expected to have the following interface:

    - def name_changed(robot):
    The user at any time may change the name of the robot in the table.  When this
    occurs an object of type Robot is passed to this function and the name property
    of the passed in robot will be the new name.

    - def on_save(robot):
    Called whenever the user presses the save button for a particular robot.  The
    passed robot will be of type of Robot and the contoller is expected to add another
    point to the robot's points list, update the last_save field, and return the updated
    robot object.  If an error occurs then a string indicating the error message should
    be returned which will cause an informational dialog to be presented to the user.

    - def on_reset(robot):
    Whenever the user presses the save button its number of points in the Robot object is
    incremented by one if the save was successful.  The resetting operation resets eliminates
    all previously accumulated points when the user accepts the confirmation.  The object
    of type Robot will be passed to this function that should reset the points field to the
    empty list and return the updated robot object.  Note that this method is also called
    when the user checks or unchecks the 'Aerial' checkbox as it is expected that currently
    accumulated points are not valid for a different type of robot.

    - def on_calibrate(robot):
    After collecting a number of points the user will want to finish the calibration and
    save the transformed DARPA coordinate frame. They achieve this by clicking on the
    'Calibrate' button on the UI which then calls this method.  The return value should be
    a tuple where the first element is a boolean that is True when an error has occurred
    and the second element is a string that describes the error.  If the first value of
    the returned tuple is false then the second element should be the robot with its
    updated transform frame.

    The only 'public' method exposed is the constructor which expects the
    controller and existing list of robots that may have been previously
    persisted.  That list is expected to have robots of type Robot and will be
    used to initially populate the robot table.
    """

    def __init__(self, controller, robots):
        super(CalibrationView, self).__init__()
        rp = rospkg.RosPack()
        ui = os.path.join(
            rp.get_path("basestation_gui_python"), "resources", "calibration.ui"
        )
        loadUi(ui, self, {})
        self.controller = controller
        self.setObjectName("CalibrationView")
        self.robot_table.itemChanged[QtWidgets.QTableWidgetItem].connect(
            self.name_changed
        )
        self.robot_table.setColumnWidth(AERIAL_COL, 60)
        self.robot_table.setColumnWidth(POINT_COL, 60)
        self.robot_table.setColumnWidth(SAVE_COL, 60)
        self.robot_table.setColumnWidth(RESET_COL, 60)
        self.robot_table.setColumnWidth(TRANS_COL, 85)
        for r in robots:
            self.add_robot(r)

    def add_robot(self, robot):
        tab = self.robot_table
        row = tab.rowCount()
        item = self.non_editable(robot.name)
        item.robot = robot
        points = self.non_editable(str(len(robot.points)))
        time = self.non_editable(robot.last_save)
        checkbox = QtWidgets.QCheckBox()

        # The following are callbacks for the newly added UI controls.  Since
        # adding a row creates new widgets/buttons, we need new callbacks that
        # refer to these widgets and the robot object in question as certain
        # operations want to mutate the robot.
        def update():
            points.setText(str(len(item.robot.points)))
            time.setText(item.robot.last_save)

        def reset():
            item.robot = self.controller.on_reset(item.robot)
            update()

        def on_save():
            result = self.controller.on_save(item.robot)
            if type(result) is str:
                QtWidgets.QMessageBox.critical(None, "Saved Point Failed", result)
            else:
                item.robot = result
                update()

        def on_reset():
            answer = confirm(
                "Reset Robot", "Really reset " + item.robot.name + "'s points?"
            )
            if answer == QtWidgets.QMessageBox.Yes:
                reset()

        def on_calibrate():
            (has_err, result) = self.controller.on_calibrate(item.robot)
            if has_err:
                QtWidgets.QMessageBox.critical(None, "Calibration Failed", result)
            else:
                MEAN_ERROR_THRESHOLD = 0.02

                item.robot = result
                mbox = QtWidgets.QMessageBox.information
                msg = "Mean error: {0}\n\n".format(result.mean_error)

                if result.mean_error > MEAN_ERROR_THRESHOLD:
                    mbox = QtWidgets.QMessageBox.warning
                    msg += "Error greater than {0}.  Consider re-calibrating.\n\n".format(
                        MEAN_ERROR_THRESHOLD
                    )

                M = result.transform
                row = "   {: 10.6f}  {: 10.6f}  {: 10.6f}  {: 10.6f}    \n"
                msg += "Transform matrix:\n\n"
                msg += row.format(M[0][0], M[0][1], M[0][2], M[0][3])
                msg += row.format(M[1][0], M[1][1], M[1][2], M[1][3])
                msg += row.format(M[2][0], M[2][1], M[2][2], M[2][3])
                msg += row.format(M[3][0], M[3][1], M[3][2], M[3][3])
                mbox(None, "Calibration Complete", msg)

        checkbox.setChecked(robot.is_aerial)
        checkbox.setEnabled(False)
        tab.insertRow(row)
        tab.setItem(row, NAME_COL, item)
        tab.setItem(row, POINT_COL, points)
        tab.setItem(row, TIME_COL, time)
        tab.setCellWidget(row, AERIAL_COL, checkbox)
        tab.setCellWidget(
            row,
            SAVE_COL,
            self.make_btn("stock_save", "Save pose from station", on_save),
        )
        tab.setCellWidget(
            row,
            RESET_COL,
            self.make_btn("stock_refresh", "Clear saved pose points", on_reset),
        )
        tab.setCellWidget(
            row,
            TRANS_COL,
            self.make_btn("system", "Calibrate and save DARPA transform", on_calibrate),
        )

    def name_changed(self, item):
        """
        Called whenever the user edits the name cell for a robot.

        This callback handler can not be part 'add_robot' method due to the fact
        that this handler has to be attached to the entire table itself and this
        same handler in that method will cause it to be called multiple times
        for each attached handler (each robot).  To differentiate which robot
        where are referring to the 'add_robot' will add the robot as a property
        to itself which is accessed here.  This handler will then only be set
        in the constructor for this view.
        """
        if item.column() == NAME_COL:
            robot = item.robot
            name = item.text()
            if robot.name != name:
                robot.name = name
                self.controller.name_changed(robot)

    def make_btn(self, theme, tip, callback):
        b = QtWidgets.QToolButton()
        b.setIcon(QtGui.QIcon.fromTheme(theme))
        b.clicked[bool].connect(callback)
        if tip is not "":
            b.setToolTip(tip)
        return b

    def non_editable(self, what):
        item = QtWidgets.QTableWidgetItem(what)
        flags = item.flags()
        item.setFlags(flags ^ QtCore.Qt.ItemIsEditable)
        return item
