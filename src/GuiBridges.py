#!/usr/bin/python

'''
File containing classes for interacting with ROS from the gui and the DARPA command post from the GUI
Contact: Bob DeBortoli (debortor@oregonstate.edu)
'''

import rospy
from std_msgs.msg import String
import yaml
import sys
from geometry_msgs.msg import PoseStamped
import pdb
from basestation_gui_python.msg import RadioMsg
import numpy as np
from darpa_command_post.TeamClient import TeamClient, ArtifactReport
import threading
import time

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

        #robot names should be unique
        if(len(np.unique(self.robot_names))!=len(self.robot_names)):
            raise ValueError('Not all of the robot names are unique!')

        for command in exp_params['robot_commands']:
            self.robot_commands.append(command)

        self.estop_commands = self.robot_commands[:4]

        #define radio message publisher
        self.radio_pub = rospy.Publisher('/from_gui', RadioMsg, queue_size=50) #queue_size arbitraily chosen

        #subscriber for listening to waypoint goals
        self.waypoint_topic = "move_base_simple/goal" #topic for listening to BSM-defined waypoints

        self.waypoint_listeners = []
        for i in range(len(self.robot_names)):
            self.waypoint_listeners.append(rospy.Subscriber(self.waypoint_topic, PoseStamped, self.publishWaypointGoal, ""))
            self.waypoint_listeners[-1].unregister() #don't start subscribing quite yet

        
        #read info on darpa-related commands (communication protocol, etc.)
        darpa_params = config['darpa_params']

        #artifact categories used for the gui
        self.artifact_categories = []
        for category in darpa_params['artifact_categories']:
            self.artifact_categories.append(category)

        
        
    def publishRobotCommand(self, command, robot_name, button):
        '''
        A command button has been pressed. Publish a command from the gui to the robot
        '''

        if(not button.isChecked()): #it has just be un-clicked

            if(command == "Define waypoint"):
                #find the robot name index and unsubscribe it
                try:
                    ind = self.robot_names.index(robot_name)
                    self.waypoint_listeners[ind].unregister()
                except ValueError:
                    print "Something went wrong registering robot names and the subscriber listening to waypoint definitions may not have been disabled!!"


        else: #the button has just been pressed
            if(command in self.estop_commands):
                self.publishEstop(command, robot_name)
            elif(command == "Define waypoint"):
                self.defineWaypoint(robot_name)
            elif(command == "Return home"):
                self.publishReturnHome(robot_name)
            else:
                print 'WARNING: Button pressed does not have a function call associated with it!'

    def publishEstop(self, command, robot_name):
        '''
        Publish an estop message after a button has been pressed
        '''

        radio_msg = RadioMsg()
        radio_msg.message_type = RadioMsg.MESSAGE_TYPE_ESTOP
        radio_msg.recipient_robot_id = self.robot_names.index(robot_name)
        if(command=="Resume"):
            radio_msg.data = RadioMsg.ESTOP_RESUME
        elif(command=="Pause"):
            radio_msg.data = RadioMsg.ESTOP_PAUSE
        elif(command=="Soft e-stop"):
            radio_msg.data = RadioMsg.ESTOP_SOFT
        elif(command=="Hard e-stop"):
            radio_msg.data = RadioMsg.ESTOP_HARD
        else:
            print 'WARNING: The pressed button does not correspond to an estop command the Bridge knows about'

        self.radio_pub.publish(radio_msg)

    def publishReturnHome(self, robot_name):
        '''
        Send out a message for the robot to return home
        '''
        radio_msg = RadioMsg()
        radio_msg.message_type = RadioMsg.RETURN_HOME
        self.radio_pub.publish(radio_msg)
        

    def publishWaypointGoal(self, msg, robot_name):

        radio_msg = RadioMsg()
        radio_msg.message_type = RadioMsg.MESSAGE_TYPE_DEFINE_WAYPOINT
        radio_msg.recipient_robot_id = self.robot_names.index(robot_name)
        radio_msg.data = str(msg.pose.position.x) +","+str(msg.pose.position.y) +","+str(msg.pose.position.z)+","+\
                         str(msg.pose.orientation.x) +","+str(msg.pose.orientation.y) +","+str(msg.pose.orientation.z)+","+\
                         str(msg.pose.orientation.w)

        self.radio_pub.publish(radio_msg)

        # print radio_msg.data



    def defineWaypoint(self, robot_name):
        '''
        Listen for a waypoint to be pressed in Rviz
        '''

        #subscriber for listening to waypoint goals
        try:
            self.waypoint_listeners[self.robot_names.index(robot_name)] = rospy.Subscriber(self.waypoint_topic, PoseStamped, self.publishWaypointGoal, robot_name)
        except ValueError:
            print "Something went wrong registering robot names and the subscriber listening to waypoint definitions may not have been enabled!!"


        


class DarpaGuiBridge:
    def __init__(self, config_filename):

        # parse the config file
        config = yaml.load(open(config_filename, 'r').read())
        
        #read info on the experiment parameters (# of robots, etc.)
        darpa_params = config['darpa_params']

        self.auth_bearer_token = darpa_params['auth_bearer_token'][0]
        self.request_info_uri = darpa_params['scoring_uris'][0] #uri for requesting information (time,score,etc) from darpa
        self.post_artifact_uri = darpa_params['scoring_uris'][1] #uri for posting artifact proposals to DARPA
        self.total_num_reports = darpa_params['total_num_reports'][0] #number of reports we can send to darpa in a run
        self.run_length = darpa_params['run_length'][0]

        #have an initial darpa status update
        self.darpa_status_update={}
        self.darpa_status_update['run_clock'] = None

        #setup the http client (bridge for interacting with DARPA)
        self.http_client = TeamClient()

        #if we're also simulating the darpa command post
        self.connect_to_command_post = rospy.get_param("/connect_to_command_post")

        #start a schedule which runs "get status" every few seconds
        if(self.connect_to_command_post):
            self.get_status_thread = threading.Timer(1.0, self.getStatus)
            self.get_status_thread.start()

        #define publisher for publishing when we get stuff from darpa
        self.status_pub = rospy.Publisher('/darpa_status_updates', String, queue_size=10)

    def startArtifactProposal(self, data):
        '''
        Make a thread for a call to DARPA
        '''
        #run this request on its thread
        results = self.sendArtifactProposal(data)

        return results    

    def sendArtifactProposal(self, data):
        '''
        Send artifact proposal to DARPA
        data = (x,y,z,artifact_category)
        '''
        [x, y, z, cat] = data
        artifact_report = ArtifactReport(x=x, y=y, z=z, type=cat)
        
        #run this request on its thread
        #results = []
        #thread = threading.Thread(target=self.http_client.send_artifact_report, args=(artifact_report, results))
        #thread.start()
        # thread.join()

        results = self.http_client.send_artifact_report(artifact_report)
        print "\n\nResults: ",results


        if(results==[]): #if nothing got returned from the http reqest
            return results

        artifact_report_reply, http_status, http_reason = results

        return artifact_report_reply['run_clock'], artifact_report_reply['type'], artifact_report_reply['x'], \
               artifact_report_reply['y'], artifact_report_reply['z'], \
               artifact_report_reply['report_status'], artifact_report_reply['score_change'], http_status, http_reason
        

    def getStatus(self):
        '''
        Function that calls the http code to get the status
        '''
        self.darpa_status_update = self.http_client.get_status_from_command_post()

        #publish this data as something
        time_remaining = self.displaySeconds(float(self.run_length) - float(self.darpa_status_update['run_clock']))

        msg = String()
        
        msg.data = str("Time Left: "+str(time_remaining)+'   //   Score: '+\
                   str(self.darpa_status_update['score'])+ '   //   Remaining Reports: '+\
                   str(self.darpa_status_update['remaining_reports'])+'/'+str(self.total_num_reports))

        self.status_pub.publish(msg)

        #restart the thread to call this function again
        self.get_status_thread = threading.Timer(1.0, self.getStatus)
        self.get_status_thread.start()

    def shutdownHttpServer(self):
        '''
        Closes the http connection
        '''

        #stop the thread that keeps reuesting the status
        self.get_status_thread.cancel()


        self.http_client.exit()

    def displaySeconds(self, seconds):
        '''
        Function to convert seconds float into a min:sec string
        '''
        seconds_int = int(seconds-(int(seconds)/60)*60)
        if seconds_int < 10:
            seconds_str = '0'+str(seconds_int)
        else:
            seconds_str = str(seconds_int)
        return str((int(seconds)/60))+':'+seconds_str






        

if __name__ == '__main__':
    rospy.init_node('ros_gui_bridge', anonymous=True)
    
    config_filename = rospy.get_param('~config', '')

    ros_gui_bridge = RosGuiBridge(config_filename)    

    rospy.spin()
