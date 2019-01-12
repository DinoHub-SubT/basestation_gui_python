#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

class GuiApp:
    '''
    Place where the GUI code is help for sending commands to robots and displaying their information
    '''
    def __init__(self, bot_labels):

    	self.bot_labels = bot_labels
    	self.num_bots = len(bot_labels) #number of robots

    	#setup the various pubs/subs
        self.setupPubs()
        self.setupSubs()

    def setupPubs(self):
        self.estop_pubs = []
        #make custom publishers for each robot
        for robot_label in self.bot_labels:
            self.estop_pubs.append(rospy.Publisher('/'+robot_label+'/e_stop', Int32, queue_size=10))

    def setupSubs(self):
        for robot_label in self.bot_labels:
            rospy.Subscriber('/'+robot_label+'/status_battery', Int32, callback)
            rospy.Subscriber('/'+robot_label+'/status_mobility', Int32, callback)
            rospy.Subscriber('/'+robot_label+'/status_comms', Int32, callback)

    	