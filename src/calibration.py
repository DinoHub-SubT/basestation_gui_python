from __future__ import print_function

import os
import random
import datetime
import uuid
import json
import re
import subprocess
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore, QtGui, QtWidgets
from nav_msgs.msg import Odometry

class Robot(object):
    """
    Robot provides a structure for the UI to communicate with its controller.  Typically,
    the UI will create Robot objects when the user clicks the 'Add Robot' button and the
    view will update the controller to manipulate this structure as necessary.  Explanation
    of the following fields:

    - name
    Self-explanatory.  Note that names do not need to be unique though it may be confusing
    if two robots have the same name.

    - is_aerial
    True if the robot is an aerial (drone) robot.

    - points
    Number of points saved so far.

    - last_save
    Records the datetime of when either the last successful point save operation has
    been performed on the robot in the UI.  This should be reset to the empty string
    if the user resets the points.

    - uuid
    A random unique identifier that conforms to the UUID4 specification.  This created
    once and should never be changed.

    - mean_error
    The computed mean error after calibrating the transform matrix.

    - transform:
    A 4x4 matrix that can transform CMU's Subt coordinate frame to DARPA's coordinate
    frame.  The default is the identity matrix.
    """
    def __init__(self, name):
        self.name       = name
        self.is_aerial  = False
        self.points     = []
        self.last_save  = ''
        self.uuid       = uuid.uuid4()
        self.mean_error = 0
        self.transform  = [[1,0,0,0],
                           [0,1,0,0],
                           [0,0,1,0],
                           [0,0,0,1]]

    def encode_to_json(self):
        '''Encode this Robot object into a dictionary that is suitable for JSON encoding.'''
        d = dict()
        d['name']       = self.name
        d['uuid']       = str(self.uuid)
        d['is_aerial']  = self.is_aerial
        d['points']     = self.points
        d['last_save']  = self.last_save
        d['mean_error'] = self.mean_error
        d['transform']  = self.transform
        return d

    @staticmethod
    def decode_from_json(obj):
        '''Decode a dictionary object that was loaded from JSON and return a new Robot object.'''
        r = Robot(obj['name'])
        r.uuid       = uuid.UUID(obj['uuid'])
        r.is_aerial  = obj['is_aerial']
        r.points     = obj['points']
        r.last_save  = obj['last_save']
        r.mean_error = obj['mean_error']
        r.transform  = obj['transform']
        return r


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
        self.setObjectName('Calibration')

        # Load previously persisted robots.  Note that this is not robust in the sense
        # that no I/O errors are checked for, exceptions caught, or checked that someone
        # has tampered with the persisted JSON files in the data directory.  For a program
        # exposed to a large number of users such protections should be put into place;
        # however, given that the number of users for the basestation is expected to be
        # less than five and are trained in its usage, we forego such error checking and
        # favor simplicity in the implementation.
        robots = []
        root = self.robot_dir()
        for fn in os.listdir(root):
            if fn.endswith(".json"):
                path = os.path.join(root, fn)
                with open(path) as f:
                    r = Robot.decode_from_json(json.load(f))
                    robots.append(r)

        self.view = CalibrationView(self, robots)
        context.add_widget(self.view)

        self.last_pose_ground = None
        self.last_pose_aerial = None
        self.last_pose_total = None

        def odometry_to_pose(odometry):
            return [
                odometry.pose.pose.position.x,
                odometry.pose.pose.position.y,
                odometry.pose.pose.position.z
            ]

        # Subscribe callbacks
        def on_pose_ground(msg):
            self.last_pose_ground = odometry_to_pose(msg)

        def on_pose_aerial(msg):
            self.last_pose_aerial = odometry_to_pose(msg)

        def on_pose_total( msg):
            self.last_pose_total = odometry_to_pose(msg)

        rospy.Subscriber('/ugv1/integrated_to_map', Odometry, on_pose_ground)
        rospy.Subscriber('/uav1/integrated_to_map', Odometry, on_pose_aerial)
        rospy.Subscriber('/position', Odometry, on_pose_total)

    #################### private methods ####################
    def add_pose(self, robot, pose, total):
        robot.points.append([pose, total])
        robot.last_save = str(datetime.datetime.now())
        self.persist(robot)
        return robot

    def robot_dir(self):
        '''Robot_dir returns the directory where Robot objects are archived.'''
        return os.path.join(rospkg.RosPack().get_path('basestation_gui_python'), 'data/calibration')

    def robot_filename(self, robot):
        '''Robot_filename returns the full file path of where a Robot object should be archived.'''
        return os.path.join(self.robot_dir(), str(robot.uuid) + ".json")

    def persist(self, robot):
        '''Persist archives _robot_ as json to the path specificed by robot_filename.'''
        fn = self.robot_filename(robot)
        with open(fn, 'w') as f:
            json.dump(robot.encode_to_json(), f)

    #################### CalibrationView interface methods ####################
    def new_robot(self, robot):
        self.persist(robot)

    def name_changed(self, robot):
        self.persist(robot)

    def on_save(self, robot):
        have_ground = self.last_pose_ground != None
        have_aerial = self.last_pose_aerial != None
        have_total = self.last_pose_total != None
        if robot.is_aerial:
            if have_aerial and have_total:
                return self.add_pose(robot, self.last_pose_aerial, self.last_pose_total)
            elif have_aerial:
                return "Have not received total pose.  No point saved."
            else:
                return "Have not received aerial pose.  No point saved."
        else:
            if have_ground and have_total:
                return self.add_pose(robot, self.last_pose_ground, self.last_pose_total)
            elif have_ground:
                return "Have not received total pose.  No point saved."
            else:
                return "Have not received ground pose.  No point saved."

    def on_delete(self, robot):
        os.remove(self.robot_filename(robot))

    def on_reset(self, robot):
        robot.points = []
        robot.last_save = ''
        self.persist(robot)
        return robot

    def on_calibrate(self, robot):
        RPKG = rospkg.RosPack()
        ECAL = 'entrance_calib'
        if ECAL not in RPKG.list():
            m = "The ROS '{0}' package was not found in the package list.  Ensure that it is installed and sourced.".format(ECAL)
            return (True, m)
        if len(robot.points) < 3:
            # This error was from reading the calibration code and we perform it here in
            # order to get a fast and clear error message to the user.  The drawback is
            # that if someone modifies the calibration process and changes the number of
            # points required for a calibration then this has to change as well.  C'est la vie.
            return (True, "Not enough points.  Need 3 or more points for a calibration.")

        # In a previous implementation the GUI program would record the data into the
        # calibration's data folder on the go while keeping separate files for the ground and
        # aerial vehicle.  Here we choose a different strategy where we already have the
        # necessary points cached in memory and persisted in our JSON files so we just
        # dump the output to the same location where the calibration process expects to
        # find the data.  Note that we always dump to the exact output location regardless
        # if the robot is a UGV or UAV as the calibration makes no differentiation in the
        # matter.  This keeps us from having to modify ROS parameters to tell the calibration
        # which files to load and just its defaults listed in its launch file.
        cal_data = os.path.join(RPKG.get_path('entrance_calib'), 'data')
        est_path = os.path.join(cal_data, 'ugv0_state_estimation.txt')
        tot_path = os.path.join(cal_data, 'ugv0_total_station.txt')
        with open(est_path, 'w') as est:
            with open(tot_path, 'w') as tot:
                this = '{0}\n'.format(len(robot.points))
                est.write(this)
                tot.write(this)
                # Example of what robot.points looks like:
                #    [
                #      [ [1,2,3], [4,5,6] ],
                #      ...
                #      [ [1,2,3], [4,5,6] ],
                #    ]
                for e, t in robot.points:
                    e_pt = '{0} {1} {2}\n'.format(e[0], e[1], e[2])
                    t_pt = '{0} {1} {2}\n'.format(t[0], t[1], t[2])
                    est.write(e_pt)
                    tot.write(t_pt)

        # Call the actual calibration process.  Previously this step had to be done manually
        # as the user had to use both the terminal and GUI in conjuction with each other
        # to perform the calibration and upload the transformed frame to DARPA.
        out = ''
        try:
            out = subprocess.check_output(['roslaunch', ECAL, ECAL + '.launch'], stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError as e:
            bad = str(e) + ':\n\n' + out
            rospy.logerr(bad)
            return (True, bad)
        # This check is for the case when subprocess.CalledProcessError is not thrown.
        # A test was made to have 'entrance_calib' exit with a code of one by having one
        # of the files it was looking for not exist.  When doing so the exception was
        # thrown and it apppeared that ROS was 'swallowing' the exit code itself and
        # returning a new code of zero.
        if 'exit code 1' in out:
            rospy.logerr(out)
            return (True, "'entrance_calib' has shutdown unexpectedly:\n\n" + out)

        # At this point we have successfully run the calibration and are now able
        # to proceed on transforming its results and save the DARPA transform.

        # Extract the mean error from the process standard output.
        match = re.search("Mean error:\s*([.\w]+)", out, flags=re.MULTILINE|re.IGNORECASE)
        if not match:
            return (True, "No mean error detected in calibration output.")
        mean_error = float(match.group(1)) # save for later in case of other errors

        cal_path = os.path.join(cal_data, 'calib.txt')
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
            return (True, "Calibration output is missing necessary matrix data.  Transform unchanged.")

        lines = [line.strip() for line in lines] # strip newlines
        # Note that lines[3] is the empty string.
        r0 = map(float, lines[0].split(' '))
        r1 = map(float, lines[1].split(' '))
        r2 = map(float, lines[2].split(' '))
        r0.append(float(lines[4]))
        r1.append(float(lines[5]))
        r2.append(float(lines[6]))
        if len(r0) < 4 and len(r1) < 4 and len(r2) < 4:
            return (True, "Calibration output is missing necessary data.  Transform unchanged.")

        robot.transform[0] = r0
        robot.transform[1] = r1
        robot.transform[2] = r2
        robot.mean_error = mean_error
        self.persist(robot)
        return (False, robot)


# The following constants represent the columns in the table of the calibration UI.
NAME_COL   = 0
AERIAL_COL = 1
POINT_COL  = 2
SAVE_COL   = 3
DELETE_COL = 4
RESET_COL  = 5
TRANS_COL  = 6
TIME_COL   = 7

def confirm(title, message):
    """
    Presents a simple message box to the asking to confirm the operation.

    Returns QtWidgest.QMessageBox.Yes if the user confirms; otherwise, returns
    QtWidgest.QMessageBox.Cancel.
    """
    MB = QtWidgets.QMessageBox
    return MB.warning(None, title, message, buttons=MB.Yes|MB.Cancel, defaultButton=MB.Yes)


class CalibrationView(QtWidgets.QWidget):
    """
    Everything UI to calibrate a robot and transforms its coordinate frame to
    be uploaded to DARPA.  The controller, the second argument to the constructor
    is expected to have the following interface:

    - def new_robot(robot):
    Called whenever the user adds a new robot to the UI.  A default Robot object
    is created with a randomly assigned name.

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

    - def on_delete(robot):
    The user may delete a robot at any time from the table.  They will be first
    presented with a confirmation dialog and if responding yes the robot in questions
    will be passed to the function.  There is no response required from the controller
    as the robot will also be removed from the table.

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
        ui = os.path.join(rp.get_path('basestation_gui_python'), 'resources', 'calibration.ui')
        loadUi(ui, self, {})
        self.controller = controller
        self.setObjectName('CalibrationView')
        self.add_robot_button.clicked[bool].connect(self.add_robot_clicked)
        self.robot_table.itemChanged[QtWidgets.QTableWidgetItem].connect(self.name_changed)
        self.robot_table.setColumnWidth(AERIAL_COL, 60)
        self.robot_table.setColumnWidth(POINT_COL, 60)
        self.robot_table.setColumnWidth(SAVE_COL, 60)
        self.robot_table.setColumnWidth(DELETE_COL, 65)
        self.robot_table.setColumnWidth(RESET_COL, 60)
        self.robot_table.setColumnWidth(TRANS_COL, 85)
        for r in robots:
            self.add_robot(r)

    def add_robot_clicked(self):
        n = random.choice([ 'Rocket', 'Gamora', 'Peter', 'Drax', 'Groot',
                            'Nebula', 'Mantis', 'Yondu', 'Taserface', 'Thanos' ])
        r = Robot(n)
        self.add_robot(r)
        self.controller.new_robot(r)

    def add_robot(self, robot):
        tab        = self.robot_table
        row        = tab.rowCount()
        item       = QtWidgets.QTableWidgetItem(robot.name)
        item.robot = robot
        points     = self.non_editable(str(len(robot.points)))
        time       = self.non_editable(robot.last_save)
        checkbox   = QtWidgets.QCheckBox()

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
                QtWidgets.QMessageBox.critical(None, 'Saved Point Failed', result);
            else:
                item.robot = result
                update()

        def on_reset():
            answer = confirm("Reset Robot", "Really reset " + item.robot.name + "'s points?")
            if answer == QtWidgets.QMessageBox.Yes:
                reset()

        def on_delete():
            answer = confirm("Remove Robot", "Really delete the robot " + item.robot.name + "?")
            if answer == QtWidgets.QMessageBox.Yes:
                row = self.robot_table.row(item)
                self.robot_table.removeRow(row)
                self.controller.on_delete(item.robot)

        def on_calibrate():
            (has_err, result) = self.controller.on_calibrate(item.robot)
            if has_err:
                QtWidgets.QMessageBox.critical(None, 'Calibration Failed', result)
            else:
                MEAN_ERROR_THRESHOLD = 0.02

                item.robot = result
                mbox = QtWidgets.QMessageBox.information
                msg  = "Mean error: {0}\n\n".format(result.mean_error)

                if result.mean_error > MEAN_ERROR_THRESHOLD:
                    mbox = QtWidgets.QMessageBox.warning
                    msg += "Error greater than {0}.  Consider re-calibrating.\n\n".format(MEAN_ERROR_THRESHOLD)

                M    = result.transform
                row  = "   {: 10.6f}  {: 10.6f}  {: 10.6f}  {: 10.6f}    \n"
                msg += "Transform matrix:\n\n"
                msg += row.format(M[0][0], M[0][1], M[0][2], M[0][3])
                msg += row.format(M[1][0], M[1][1], M[1][2], M[1][3])
                msg += row.format(M[2][0], M[2][1], M[2][2], M[2][3])
                msg += row.format(M[3][0], M[3][1], M[3][2], M[3][3])
                mbox(None, 'Calibration Complete', msg)

        def on_aerial_checked(checked):
            if checked == 0:
                item.robot.is_aerial = False
            else:
                item.robot.is_aerial = True
            reset()

        checkbox.stateChanged[int].connect(on_aerial_checked)
        tab.insertRow(row)
        tab.setItem(row, NAME_COL, item)
        tab.setItem(row, POINT_COL, points)
        tab.setItem(row, TIME_COL, time)
        tab.setCellWidget(row, AERIAL_COL, checkbox)
        tab.setCellWidget(row, SAVE_COL, self.make_btn('stock_save', 'Save pose from station', on_save))
        tab.setCellWidget(row, DELETE_COL, self.make_btn('stock_delete', 'Delete this robot', on_delete))
        tab.setCellWidget(row, RESET_COL, self.make_btn('stock_refresh', 'Clear saved pose points', on_reset))
        tab.setCellWidget(row, TRANS_COL, self.make_btn('system', 'Calibrate and save DARPA transform', on_calibrate))

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
        if tip is not '':
            b.setToolTip(tip)
        return b

    def non_editable(self, what):
        item = QtWidgets.QTableWidgetItem(what)
        flags = item.flags()
        item.setFlags(flags ^ QtCore.Qt.ItemIsEditable)
        return item
