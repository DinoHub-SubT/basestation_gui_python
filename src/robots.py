"""
Robots module contains components necessray to load the GUI setup parameters.  Such
parameters are the number of robots and the specific configuration for each, DARPA
connection information, etc.
"""
import yaml
import os
import rospy
import rospkg


class Config(object):
    """
    Config is responsible for loading the main configuration file for the base station
    GUI.  The configuration YAML file is expected to reside inside the 'config' directory
    under the 'basestation_gui_python' project.  The name of the file is set within the
    'gui.launch' file under the 'gui_setup' ROS parameter.  Config exposes the following
    properties:

    robots - A list of all loaded robots, represented as Robot objects, from the YAML file.

    darpa - The darpa information from the YAML file.  See the Darpa object in this module.
    """

    def __init__(self):
        self.robots = []
        self.darpa = Darpa()

        pkg = rospkg.RosPack().get_path("basestation_gui_python")
        gui = rospy.get_param("gui_setup")
        cfg = os.path.join(pkg, "config", gui)
        with open(cfg) as f:
            config = yaml.load(f)

        # We set this as a list in order for the 'require' function below to
        # set an 'error' when it is called.  A cleaner representation would
        # be for 'require' to return a tuple but that would require separate
        # variables that need to be checked for each 'require' call.
        req_err = [False]

        def require(d, key, struct):
            if d.has_key(key):
                return d.get(key)
            else:
                m = "[Base Station GUI] Missing '%s' property in %s structure of %s"
                rospy.logerr(m, key, struct, gui)
                req_err[0] = True
                return None

        def require_topic(topics, key):
            if req_err[0] == False and topics.has_key(key) == False:
                req_err[0] = True
                rospy.logerr("[Base Station GUI] Missing '%s' topic in topics", key)

        def option(d, what):
            return d.has_key(what) and str(d.get(what)).lower() == "true"

        for r in config["robots"]:
            req_err[0] = False
            robot = Robot()
            robot.name = require(r, "name", "robot")
            robot.uuid = require(r, "uuid", "robot")
            robot.estop_serial_port = require(r, "estop_serial_port", "robot")
            robot.estop_engage = require(r, "estop_engage", "robot")
            robot.estop_disengage = require(r, "estop_disengage", "robot")
            robot.topic_prefix = require(r, "topic_prefix", "robot")
            robot.topics = require(r, "topics", "robot")
            require_topic(robot.topics, "odometry")
            require_topic(robot.topics, "calibration")
            require_topic(robot.topics, "darpa_tf")
            require_topic(robot.topics, "wifi_detection")
            require_topic(robot.topics, "status_update")
            require_topic(robot.topics, "radio_command")
            require_topic(robot.topics, "behavior_tree")
            require_topic(robot.topics, "point_cloud")
            require_topic(robot.topics, "clear_cloud")
            require_topic(robot.topics, "complete_cloud")
            if req_err[0]:
                continue
            else:
                engage = robot.estop_engage.split()
                disengage = robot.estop_disengage.split()
                robot.estop_engage = "".join(engage).decode("hex")
                robot.estop_disengage = "".join(disengage).decode("hex")
            robot.is_aerial = option(r, "is_aerial")
            robot.has_comms = option(r, "has_comms")
            if r.has_key("max_travel_time"):
                robot.max_travel_time = int(r.get("max_travel_time"))
            if r.has_key("max_flight_speed"):
                robot.max_flight_speed = float(r.get("max_flight_speed"))
            self.robots.append(robot)

        darpaCfg = config["darpa"]
        darpa = Darpa()
        darpa.score_address = require(darpaCfg, "score_address", "darpa")
        darpa.score_port = require(darpaCfg, "score_port", "darpa")
        darpa.score_bearer_token = require(darpaCfg, "score_bearer_token", "darpa")
        darpa.score_status_uri = require(darpaCfg, "score_status_uri", "darpa")
        darpa.score_report_uri = require(darpaCfg, "score_report_uri", "darpa")
        darpa.map_address = require(darpaCfg, "map_address", "darpa")
        darpa.map_port = require(darpaCfg, "map_port", "darpa")
        darpa.map_bearer_token = require(darpaCfg, "map_bearer_token", "darpa")
        darpa.map_cloud_uri = require(darpaCfg, "map_cloud_uri", "darpa")
        darpa.map_pose_uri = require(darpaCfg, "map_pose_uri", "darpa")
        darpa.artifact_categories = require(darpaCfg, "artifact_categories", "darpa")
        if not req_err[0]:
            self.darpa = darpa


class Robot(object):
    """
    Robot is the Python representation of a robot structure within the setup
    YAML file.  Its use is to customize the GUI depending on assigned properties.
    """

    def __init__(self):
        self.name = None
        self.uuid = None
        self.is_aerial = False
        self.has_comms = False
        self.max_travel_time = 10
        self.max_flight_speed = 1.0
        self.estop_serial_port = None
        self.estop_engage = None
        self.estop_disengage = None
        self.topic_prefix = None
        self.topics = dict()

    def __repr__(self):
        return str(self)

    def __str__(self):
        return "[{0}] {1}: {2}".format(self.kind, self.name, self.uuid)


class Darpa(object):
    """
    Darpa is the Python representation of the DARPA information located in
    in the setup YAML file.  Its use is to the GUI communicate with DARPA's
    system.
    """

    def __init__(self):
        self.score_address = '127.0.0.1'
        self.score_port = 8080
        self.score_bearer_token = None
        self.score_status_uri = None
        self.score_report_uri = None
        self.map_address = '127.0.0.1'
        self.map_port = 8080
        self.map_bearer_token = None
        self.map_cloud_uri = None
        self.map_pose_uri = None
        self.artifact_categories = []

    def __repr__(self):
        return str(self)

    def __str__(self):
        m = "[DARPA] Score Token: {0}, Map Token: {1}"
        return m.format(self.score_bearer_token, self.map_bearer_token)
