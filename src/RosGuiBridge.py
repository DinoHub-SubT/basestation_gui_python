#!/usr/bin/python
import rospy
from std_msgs.msg import String
import yaml
import sys

import pdb


class RosGuiBridge:
    def __init__(self, config_filename):

        # parse the config file
        config = yaml.load(open(config_filename, 'r').read())
        
        #read info on the experiment parameters (# of robots, etc.)
        exp_params = config['experiment_params']
        self.robot_names = [] #robot names corresponding to topic published
        self.robot_commands = [] #different types of estop commands

        for name in exp_params['robot_names']:
            self.robot_names.append(name)

        for command in exp_params['robot_commands']:
            self.robot_commands.append(command)
        

if __name__ == '__main__':
    rospy.init_node('ros_gui_bridge', anonymous=True)
    
    config_filename = rospy.get_param('~config', '')

    pdb.set_trace()
    
    ros_gui_bridge = RosGuiBridge(config_filename)    

    rospy.spin()
