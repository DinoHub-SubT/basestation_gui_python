#!/usr/bin/python

'''
File containing classes for interacting with ROS from the gui and the DARPA command post from the GUI
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University (CMU) / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebation or Matt).
'''

import rospy
from std_msgs.msg import String, Float32MultiArray
import yaml
import sys
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
import pdb
from basestation_gui_python.msg import RadioMsg, NineHundredRadioMsg
import numpy as np
from darpa_command_post.TeamClient import TeamClient, ArtifactReport
import threading
import time
from visualization_msgs.msg import InteractiveMarkerFeedback, MarkerArray, Marker
import math
import tf
from functools import partial

class RosGuiBridge:
    def __init__(self, config_filename, gui):

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

        #remapping aerial commands
        self.remap_to_aerial_commands = {}
        self.remap_from_aerial_commands = {}


        for i, name in enumerate(exp_params['aerial_commands']):
            self.remap_from_aerial_commands[name] =  self.robot_commands[i]
            self.remap_to_aerial_commands[self.robot_commands[i]] =  name

        #define radio message publisher
        self.radio_pub = rospy.Publisher('/from_gui', RadioMsg, queue_size=50) #queue_size arbitraily chosen
        self.radio_900_pub = rospy.Publisher('/ros_to_teensy', NineHundredRadioMsg, queue_size=50) #queue_size arbitraily chosen

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


        #publisher for moving the refinment marker
        self.refinement_marker_pos_pub = rospy.Publisher('/refinement_marker_pos', Point, queue_size=50)

        #publisher for turning the marker off
        self.refinement_marker_off_pub = rospy.Publisher('/refinement_marker_off', Point, queue_size=50)

        [self.highlight_robot_marker,  self.ugv_bluetooth_marker, self.uav_bluetooth_marker] = self.initMarkers()
        
        #publisher for displaying the original position        
        self.marker_orig_pos_pub = rospy.Publisher('/refinement_marker_orig_pos', MarkerArray, queue_size=50)

         #publisher for displaying the original position        
        self.highlight_robot_pub = rospy.Publisher('/highlight_robot_pub', Marker, queue_size=50)

        #publisher for displaying the original position        
        self.bluetooth_marker_pub_ugv = rospy.Publisher('/ugv/bluetooth_marker', Marker, queue_size=50)

        #publisher for displaying the original position        
        self.bluetooth_marker_pub_uav = rospy.Publisher('/uav/bluetooth_marker', Marker, queue_size=50)

        #publisher for the image coordinate clicked on (as percentage of the image)
        self.img_coord_pub = rospy.Publisher('/gui_img_clicked', Float32MultiArray, queue_size=10)

        #thread for publishing drone hard estop
        self.drone_hard_estop_thread = threading.Timer(2.0, partial(self.persistentDroneHardEstop, self.robot_names[1])) 

    def persistentDroneHardEstop(self, robot_name):
        '''
        Publish a hard estop every __ seconds for the drone
        '''
        radio_900_msg = NineHundredRadioMsg()
        radio_900_msg.message_type = 1
        radio_900_msg.recipient_robot_id = self.robot_names.index(robot_name)
        self.radio_900_pub.publish(radio_900_msg)
        radio_msg = RadioMsg()
        radio_msg.message_type = RadioMsg.MESSAGE_TYPE_ESTOP
        radio_msg.recipient_robot_id = self.robot_names.index(robot_name)
        radio_msg.data = RadioMsg.ESTOP_HARD
        self.radio_pub.publish(radio_msg)

        for cmd_button in self.gui.control_buttons[self.robot_names.index(robot_name)]:
            if (cmd_button.text()=='Hard e-stop') and (cmd_button.isChecked()):
                self.drone_hard_estop_thread = threading.Timer(2.0, partial(self.persistentDroneHardEstop, self.robot_names[1])) 
                self.drone_hard_estop_thread.start()


    def initSubscribers(self, gui):
        '''
        Called after all of the gui has been built. Avoid errors where the gui subscribes to something
        and tried to populate na element (e.g. table) that does not exist
        '''

        #subscriber for updating the corresponding textboxes for the refinement marker
        self.gui = gui
        rospy.Subscriber('/basic_controls/feedback', InteractiveMarkerFeedback, self.gui.updateRefinmentPos)

        #subscriber for saving the robot pose
        self.robot_pos_ground = None
        rospy.Subscriber('/ugv1/integrated_to_map', Odometry, self.saveRobotPoseGround)

        #subscriber for saving the robot pose
        self.robot_pos_aerial = None
        rospy.Subscriber('/uav1/integrated_to_map', Odometry, self.saveRobotPoseAerial)

        #subscriber for saving the total station data
        self.total_pos = None
        rospy.Subscriber('/position', Odometry, self.saveTotalPose)

        #subscriber for listening to messages
        rospy.Subscriber('/gui_message_listener', String, self.gui.addMessage)



    def publishImageCoord(self, event):
        '''
        Publish image coordinates clicked on to ROS
        ''' 
        # self.artifact_img_width, self.artifact_img_length
        x_coord = event.pos().x() / float(self.gui.artifact_img_width)
        y_coord = event.pos().y() / float(self.gui.artifact_img_length)
        
        msg = Float32MultiArray()
        msg.data = [x_coord, y_coord]

        self.img_coord_pub.publish(msg)


    def initMarkers(self):
        '''
        Initialize and return the various markers
        '''  
        marker_list = []
        for i in range(3):
            marker = Marker()
            marker.type = Marker.SPHERE
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3

            if (i <1): #non-bluetooth markers
                marker.color.r = 0.
                marker.color.g = 1.
                marker.color.b = 0.
                marker.color.a = 1.0

            else: #bluetooth markers
                marker.color.r = 1.
                marker.color.g = 165./255.
                marker.color.b = 0.
                marker.color.a = 0.

                marker.pose.position.x = 0.
                marker.pose.position.y = 0.
                marker.pose.position.z = 0.

            
            marker.header.frame_id = '/map'

            marker_list.append(marker)

        return marker_list

    def saveRobotPoseGround(self, msg):
        '''
        Function to save the robot pose for Ji's button
        '''
        self.robot_pos_ground = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]

    def getRobotPoseGround(self):
        return self.robot_pos_ground


    def saveRobotPoseAerial(self, msg):
        '''
        Function to save the robot pose for Ji's button
        '''
        self.robot_pos_aerial = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]

    def getRobotPoseAerial(self):
        return self.robot_pos_aerial


    def saveTotalPose(self, msg):
        '''
        Function to save the robot pose for Ji's button
        '''
        self.total_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]

    def getTotalPose(self):
        return self.total_pos


        
    def publishRobotCommand(self, command, robot_name, button):
        '''
        A command button has been pressed. Publish a command from the gui to the robot
        '''

        if (command in [ "Return home", "Highlight robot", "Drop comms"]): #buttons are not checkable
            if(command == "Return home"):
                self.publishReturnHome(robot_name)
            elif(command == "Highlight robot"):
                self.highlightRobot(robot_name)
            elif(command == "Drop comms"):
                self.dropComms(robot_name)
            else:
                print self.gui.printMessage('WARNING: Button pressed does not have a function call associated with it!', self.gui.red_message) #checked

        elif(not button.isChecked()): #it has just be un-clicked

            if(command == "Define waypoint"):
                #find the robot name index and unsubscribe it
                try:
                    ind = self.robot_names.index(robot_name)
                    self.waypoint_listeners[ind].unregister()
                except ValueError:
                    print self.gui.printMessage("Something went wrong registering robot names and the subscriber listening to waypoint definitions may not have been disabled!!", self.gui.red_message)

            elif(command == "Show bluetooth"):
                self.handleBluetooth(robot_name, button)

            elif(command == "Land in comms"):
                self.pubLandInComms(robot_name, button)

        else: #the button has just been pressed
            if(command in self.estop_commands):
                self.publishEstop(command, robot_name)

            elif(command == "Define waypoint"):
                self.defineWaypoint(robot_name)


            elif(command == "Show bluetooth"):
                self.handleBluetooth(robot_name, button)

            elif(command == "Land in comms"):
                self.pubLandInComms(robot_name, button)

            else:
                print self.gui.printMessage('WARNING: Button pressed does not have a function call associated with it!', self.gui.red_message) #checked

    def pubLandInComms(self, robot_name, button):
        '''
        Depending on if the button is pressed or not, send a 
        message to land in comms range or back at home
        '''

        radio_msg = RadioMsg()
        radio_msg.recipient_robot_id = self.robot_names.index(robot_name)
        radio_msg.message_type = RadioMsg.MESSAGE_TYPE_LANDING_BEHAVIOR

        if (button.isChecked()):             
            radio_msg.data = RadioMsg.LAND_IN_COMMS

        else:            
            radio_msg.data = RadioMsg.LAND_AT_HOME

        print radio_msg.data=='comms'

        self.radio_pub.publish(radio_msg)



    def handleBluetooth(self, robot_name, button):
        '''
        Make the bluetooth marker visualizable or hidden
        '''


        if (robot_name == 'Ground1'):

            if(not button.isChecked()):
                #change the alpha of the orig_pos marker to make it invisible
                self.ugv_bluetooth_marker.color.a = 0.
            else:
                self.ugv_bluetooth_marker.color.a = 1.

            self.bluetooth_marker_pub_ugv.publish(self.ugv_bluetooth_marker)

        elif (robot_name == 'Aerial1'):
            
            if(not button.isChecked()):
                #change the alpha of the orig_pos marker to make it invisible
                self.uav_bluetooth_marker.color.a = 0.
            else:
                self.uav_bluetooth_marker.color.a = 1.

            self.bluetooth_marker_pub_uav.publish(self.uav_bluetooth_marker)
        

    def dropComms(self, robot_name):
        '''
        Send out a message to drop a commas node
        '''
        radio_msg = RadioMsg()
        radio_msg.message_type = RadioMsg.MESSAGE_TYPE_DROP_COMMS
        radio_msg.recipient_robot_id = self.robot_names.index(robot_name)
        self.radio_pub.publish(radio_msg)

    def highlightRobot(self, robot_name):
        '''
        Publish an arrow pointng to the robot position
        '''

        pass




    def publishEstop(self, command, robot_name):
        '''
        Publish an estop message after a button has been pressed
        '''

        radio_msg = RadioMsg()
        send_900 = bool(False)
        radio_900_msg = NineHundredRadioMsg()
        radio_msg.message_type = RadioMsg.MESSAGE_TYPE_ESTOP
        radio_msg.recipient_robot_id = self.robot_names.index(robot_name)
        radio_900_msg.recipient_robot_id = self.robot_names.index(robot_name)
        if(command==self.estop_commands[1]):
            radio_msg.data = RadioMsg.ESTOP_RESUME
            radio_900_msg.message_type = 0
            send_900 = True
        elif(command==self.estop_commands[0]):
            radio_msg.data = RadioMsg.ESTOP_PAUSE
        elif(command==self.estop_commands[2]):
            radio_msg.data = RadioMsg.ESTOP_SOFT
        elif(command==self.estop_commands[3]):
            radio_msg.data = RadioMsg.ESTOP_HARD
            radio_900_msg.message_type = 1
            send_900 = True
            #make the estop persistent for the drone
            if (robot_name.find('erial') != -1):
                self.drone_hard_estop_thread = threading.Timer(2.0, partial(self.persistentDroneHardEstop, self.robot_names[1]))
                self.drone_hard_estop_thread.start()
        else:
            print 'WARNING: The pressed button does not correspond to an estop command the Bridge knows about'

        if send_900:
            self.radio_900_pub.publish(radio_900_msg)

        self.radio_pub.publish(radio_msg)

    def publishReturnHome(self, robot_name):
        '''
        Send out a message for the robot to return home
        '''
        radio_msg = RadioMsg()
        radio_msg.message_type = RadioMsg.MESSAGE_TYPE_RETURN_HOME
        radio_msg.recipient_robot_id = self.robot_names.index(robot_name)
        self.radio_pub.publish(radio_msg)

    def publishRefinementMarkerPos(self, artifact, button):
        '''
        Publish a position change to the refinement marker
        '''
        if(not button.isChecked()):
            #change the alpha of the orig_pos marker to make it invisible
            # self.orig_pos_marker.color.a = 0.

            for marker in self.marker_arr.markers:
                marker.color.a = 0.

            self.marker_orig_pos_pub.publish(self.marker_arr)

            #publish a bogus message to put something on this topic to turn off the refinment marker
            pose = Point(artifact.pos[0], artifact.pos[1], artifact.pos[2])
            self.refinement_marker_off_pub.publish(pose)

        else:

            #publish the interactive marker
            pose = Point(artifact.pos[0], artifact.pos[1], artifact.pos[2])
            self.refinement_marker_pos_pub.publish(pose)

            #publish 2 markers: one for the actual original pos, one above it to highlight the refinement region
            self.marker_arr = MarkerArray()

            m_orig = Marker()
            m_orig.type = m_orig.SPHERE
            m_orig.action = m_orig.ADD
            m_orig.header.frame_id = '/map'
            m_orig.id = 0

            m_orig.pose.position.x = artifact.pos[0]
            m_orig.pose.position.y = artifact.pos[1]
            m_orig.pose.position.z = artifact.pos[2]

            m_orig.scale.x = 0.3
            m_orig.scale.y = 0.3
            m_orig.scale.z = 0.3

            m_orig.color.r = 0.
            m_orig.color.g = 1.
            m_orig.color.b = 0.
            m_orig.color.a = 1.

            

            self.marker_arr.markers.append(m_orig)

            m_notify = Marker()
            m_notify.type = m_notify.ARROW
            m_notify.action = m_notify.ADD
            m_notify.header.frame_id = '/map'
            m_notify.id = 1            

            m_notify.pose.position.x = artifact.pos[0]
            m_notify.pose.position.y = artifact.pos[1]
            m_notify.pose.position.z = artifact.pos[2] + 15

            quaternion = tf.transformations.quaternion_from_euler(0., math.pi/2., 0.) #roll, pitch, yaw
            #type(pose) = geometry_msgs.msg.Pose
            m_notify.pose.orientation.x = quaternion[0]
            m_notify.pose.orientation.y = quaternion[1]
            m_notify.pose.orientation.z = quaternion[2]
            m_notify.pose.orientation.w = quaternion[3]

            m_notify.scale.x = 6.
            m_notify.scale.y = 2.
            m_notify.scale.z = 2.

            m_notify.color.r = 1.
            m_notify.color.g = 0.
            m_notify.color.b = 0.
            m_notify.color.a = 1.            

            self.marker_arr.markers.append(m_notify)

            self.marker_orig_pos_pub.publish(self.marker_arr)





            # self.orig_pos_marker.color.a = 1. #turn in back to visible if it was invisible

            # self.orig_pos_marker.pose.position.x = artifact.pos[0]
            # self.orig_pos_marker.pose.position.y = artifact.pos[1]
            # self.orig_pos_marker.pose.position.z = artifact.pos[2]            

            # self.orig_pos_marker.scale.x = 0.3
            # self.orig_pos_marker.scale.y = 0.3
            # self.orig_pos_marker.scale.z = 0.3

    def adjustMaxTime(self, robot_name, max_time_box):
        '''
        Send a maxtime for the aerial vehicle to fly
        '''
        radio_msg = RadioMsg()
        radio_msg.message_type = RadioMsg.MESSAGE_TYPE_MAX_FLIGHT_TIME
        radio_msg.recipient_robot_id = self.robot_names.index(robot_name)
        radio_msg.data = str(float(max_time_box.currentText())*60)

        self.radio_pub.publish(radio_msg)


       
        

    def publishWaypointGoal(self, msg, robot_name):


        radio_msg = RadioMsg()
        radio_msg.message_type = RadioMsg.MESSAGE_TYPE_DEFINE_WAYPOINT
        radio_msg.recipient_robot_id = self.robot_names.index(robot_name)
        radio_msg.data = str(msg.pose.position.x) +","+str(msg.pose.position.y) +","+str(msg.pose.position.z)+","+\
                         str(msg.pose.orientation.x) +","+str(msg.pose.orientation.y) +","+str(msg.pose.orientation.z)+","+\
                         str(msg.pose.orientation.w)

        self.radio_pub.publish(radio_msg)

        #de-select the appropriate buttons
        for robot_list in self.gui.control_buttons:
            for cmd_button in robot_list:
                if (cmd_button.text()=='Define waypoint'):

                    cmd_button.setChecked(False)
                    ind = self.robot_names.index(robot_name)
                    self.waypoint_listeners[ind].unregister()



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
    def __init__(self, config_filename, gui):

        # parse the config file
        config = yaml.load(open(config_filename, 'r').read())
        
        #read info on the experiment parameters (# of robots, etc.)
        darpa_params = config['darpa_params']
        

        self.auth_bearer_token = darpa_params['auth_bearer_token'][0]
        self.request_info_uri = darpa_params['scoring_uris'][0] #uri for requesting information (time,score,etc) from darpa
        self.post_artifact_uri = darpa_params['scoring_uris'][1] #uri for posting artifact proposals to DARPA
        # self.total_num_reports = darpa_params['total_num_reports'][0] #number of reports we can send to darpa in a run
        # self.run_length = darpa_params['run_length'][0]


        #have an initial darpa status update
        self.darpa_status_update = {}
        self.darpa_status_update['run_clock'] = None
        self.darpa_status_update['score'] = None
        self.darpa_status_update['remaining_reports'] = None

        #if we're also simulating the darpa command post
        self.connect_to_command_post = rospy.get_param("/connect_to_command_post")            


        #start a schedule which runs "get status" every few seconds
        if(self.connect_to_command_post):
            self.http_client = TeamClient()
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
        # print "\n\nResults: ",results


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
        time_remaining = self.displaySeconds(float(self.darpa_status_update['run_clock']))

        msg = String()
        
        msg.data = "Time: "+str(time_remaining)+'\t Score: '+\
                   str(self.darpa_status_update['score'])+ '\t Remaining Reports: '+\
                   str(self.darpa_status_update['remaining_reports'])

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
        #convert strings to floats
        seconds = float(seconds)

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
