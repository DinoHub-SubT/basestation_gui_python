#!/usr/bin/python
'''
File containing all things related to the frontend of the gui (buttons, colors, panel positions, etc.)
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebation or Matt).
'''

import os
import time
import rospy
import rospkg
from std_msgs.msg import String
import numpy as np
import yaml
import collections
import threading
import random

from qt_gui.plugin import Plugin
import python_qt_binding.QtWidgets as qt
import python_qt_binding.QtCore as core
import python_qt_binding.QtGui as gui

from python_qt_binding import QT_BINDING, QT_BINDING_VERSION

try:
    from pkg_resources import parse_version
except:
    import re

    def parse_version(s):
        return [int(x) for x in re.sub(r'(\.0+)*$', '', s).split('.')]

if QT_BINDING == 'pyside':
    qt_binding_version = QT_BINDING_VERSION.replace('~', '-')
    if parse_version(qt_binding_version) <= parse_version('1.1.2'):
        raise ImportError('A PySide version newer than 1.1.0 is required.')

from python_qt_binding.QtCore import Slot, Qt, qVersion, qWarning, Signal
from python_qt_binding.QtGui import QColor, QPixmap
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy

from GuiBridges import RosGuiBridge, DarpaGuiBridge
from functools import partial
import pdb
from argparse import ArgumentParser
from GuiEngine import GuiEngine, Artifact
import csv
import cv2

from basestation_gui_python.msg import StatusPanelUpdate, ArtifactSubmissionReply, Artifact

from PyQt5.QtCore import pyqtSignal

# from geometry_msgs.msg import Point
# from interactiveMarkerProcessing import CustomInteractiveMarker


class NumericItem(qt.QTableWidgetItem):
    '''
    Class whic overwrites a pyqt table widget item in order to allow for better sorting (e.g. '2'<'100')
    '''
    def __lt__(self, other):
        return (self.data(core.Qt.UserRole) <\
                other.data(core.Qt.UserRole))


class BasestationGuiPlugin(Plugin):
    status_panel_trigger = pyqtSignal(object)
    queue_trigger = pyqtSignal(object)
    update_artifact_trigger = pyqtSignal(object)
    arthist_trigger = pyqtSignal(object)
    print_message_trigger = pyqtSignal(object, object)
    alert_about_art_removal_trigger = pyqtSignal()
    remove_artifact_trigger = pyqtSignal(object)
    update_refinement_pos_trigger = pyqtSignal(object)
    add_message_trigger = pyqtSignal(object)
    update_info_panel_trigger = pyqtSignal(object)

    def __init__(self, context):
        super(BasestationGuiPlugin, self).__init__(context)
        self.setObjectName('BasestationGuiPlugin')

        #if we're also simulating the darpa command post
        self.connect_to_command_post = rospy.get_param("/connect_to_command_post")

        
        
        self.state_sub = rospy.Subscriber('/state', String, self.state_callback)
        self.transition_pub = rospy.Publisher('/transition', String, queue_size=1)
        self.config_filename = ''
        
        self.widget = QWidget()
        self.global_widget = qt.QGridLayout()

        self.top_widget = qt.QWidget()
        self.top_layout = qt.QGridLayout()
        self.top_widget.setLayout(self.top_layout)

        self.config_button = qt.QPushButton('Open Config...')
        self.config_button.clicked.connect(self.select_config_file)
        self.top_layout.addWidget(self.config_button, 0, 0)

        self.load_from_csv = qt.QPushButton('Load from CSV...')
        self.load_from_csv.clicked.connect(self.loadFromCsv)
        self.top_layout.addWidget(self.load_from_csv, 0, 1, 1, 2)

        

        self.load_transform_button = qt.QPushButton('Load DARPA \ntransforms')
        self.load_transform_button.clicked.connect(self.loadDarpaTransform)
        self.top_layout.addWidget(self.load_transform_button, 1,2)


        self.global_widget.addWidget(self.top_widget)
        
        
        self.widget.setLayout(self.global_widget)
        context.add_widget(self.widget)

        self.artifact_proposal_lock = threading.Lock()
        self.update_queue_lock = threading.Lock()
        self.update_message_box_lock = threading.Lock()

        self.displayed_artifact = None #which artifact is currently being displayed

        self.dont_change_art_priority = False #if we click in the artifact  queue, just update the gui and nothing else
        self.dont_change_art_category = False

        self.save_count = 0 #way to ensure we don't save the gui too often

        self.artifact_image_index = 0 #the index of the image currently being displayed

        #the image size to display image artifacts
        self.artifact_img_width, self.artifact_img_length = [640, 360]

        #define the background colors for the messages
        self.green_message = [144,238,144]
        self.red_message = [250,128,114]
        self.normal_message = [220,220,220]

        

        self.status_panel_trigger.connect(self.status_panel_update_monitor)
        self.queue_trigger.connect(self.sendToQueueMonitor)
        self.update_artifact_trigger.connect(self.updateArtifactInQueueMonitor)
        self.arthist_trigger.connect(self.drawAfterProposalMonitor)
        self.print_message_trigger.connect(self.printMessageMonitor)
        self.alert_about_art_removal_trigger.connect(self.alertImgAboutRemovalMonitor)
        self.remove_artifact_trigger.connect(self.removeQueueArtifactMonitor)
        self.update_refinement_pos_trigger.connect(self.updateRefinmentPosMonitor)
        self.add_message_trigger.connect(self.addMessageMonitor)
        self.update_info_panel_trigger.connect(self.updateInfoPanelMonitor)

        #to communicate with the submission history panel
        self.submission_reply_pub = rospy.Publisher('/gui/submission_reply', ArtifactSubmissionReply, queue_size = 10) 

        self.add_new_artifact_pub = rospy.Publisher('/gui/generate_new_artifact', Artifact, queue_size = 10)


        
    def loadDarpaTransform(self):
        '''
        Load the transformation matrix for converting artifact locations into
        the darpa reference frame
        '''

        rospack = rospkg.RosPack()

        if ('entrance_calib' not in rospack.list()):
            self.printMessage('GUI DID NOT LOAD CORRECTLY. PLACE entrance_calib PACKAGE IN THIS WORKSPACE!', self.red_message) #checked
            rospy.logerr('GUI DID NOT LOAD CORRECTLY. PLACE entrance_calib PACKAGE IN THIS WORKSPACE! ' )

        else:
            #load the text file
            transform_fname = rospack.get_path('entrance_calib')+'/data/ugv0_calib.txt'

            if (os.path.isfile(transform_fname)):


                with open(transform_fname) as f:
                    content = f.readlines()

                #strip newline chars
                content = [x.strip() for x in content] 

                transform_mat = []
                for i, row in enumerate(content):
                    if(i < 3):
                        transform_mat.append(row.split(' '))

                #set it to the numpy array self.darpa_transform
                transform_mat = np.float32(transform_mat)


                #add the translation vector
                transform_mat = np.hstack((transform_mat, np.float32([content[4], content[5], content[6]]).reshape(3,1)))
                transform_mat = np.vstack((transform_mat, np.float32([0, 0, 0, 1]).reshape(1,4)))

                self.darpa_transform_ugv = transform_mat

                self.printMessage('Loaded transform for ugv from file',  self.green_message) #checked

            else:
                self.printMessage('DARPA transform not loaded. Assuming identity transform for ugv',  self.red_message)#checked
                self.darpa_transform_ugv = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])




            transform_fname = rospack.get_path('entrance_calib')+'/data/uav1_calib.txt'

            if (os.path.isfile(transform_fname)):


                with open(transform_fname) as f:
                    content = f.readlines()

                #strip newline chars
                content = [x.strip() for x in content] 

                transform_mat = []
                for i, row in enumerate(content):
                    if(i < 3):
                        transform_mat.append(row.split(' '))

                #set it to the numpy array self.darpa_transform
                transform_mat = np.float32(transform_mat)


                #add the translation vector
                transform_mat = np.hstack((transform_mat, np.float32([content[4], content[5], content[6]]).reshape(3,1)))
                transform_mat = np.vstack((transform_mat, np.float32([0, 0, 0, 1]).reshape(1,4)))

                self.darpa_transform_uav = transform_mat

                self.printMessage('Loaded transform for uav from file',  self.green_message) #checked

            else:
                self.printMessage('DARPA transform not loaded. Assuming identity transform for uav',  self.red_message)#checked
                self.darpa_transform_uav = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])






    def associateCatsWithLabels(self, labels):
        '''
        Associating the category indices that vasu sends back 
        '''
        self.label_to_cat_dict = {}
        
        for i, lab in enumerate(labels):
            self.label_to_cat_dict[lab] = self.ros_gui_bridge.artifact_categories[i]




    def saveRobotPoseGui(self, robot_num):
        '''
        When a button is pressed, its pose gets saved to a textfile
        '''

        rospack = rospkg.RosPack()

        if ('entrance_calib' not in rospack.list()):
            self.printMessage('GUI DID NOT LOAD CORRECTLY. PLACE entrance_calib PACKAGE IN THIS WORKSPACE!', self.red_message)
            rospy.logerr('GUI DID NOT LOAD CORRECTLY. PLACE entrance_calib PACKAGE IN THIS WORKSPACE! ' )

        else:

            if (self.ros_gui_bridge.robot_names[robot_num].find('ound') != -1):

                
                robot_pose_filename = rospack.get_path('entrance_calib')+'/data/ugv0_state_estimation.txt'
                total_pose_filename = rospack.get_path('entrance_calib')+'/data/ugv0_total_station.txt'

                robot_pos = self.ros_gui_bridge.getRobotPoseGround()
                total_pos = self.ros_gui_bridge.getTotalPose()

            elif (self.ros_gui_bridge.robot_names[robot_num].find('erial') != -1):


                robot_pose_filename = rospack.get_path('entrance_calib')+'/data/uav1_state_estimation.txt'
                total_pose_filename = rospack.get_path('entrance_calib')+'/data/uav1_total_station.txt'

                robot_pos = self.ros_gui_bridge.getRobotPoseAerial()
                total_pos = self.ros_gui_bridge.getTotalPose()

            else:
                self.printMessage('Robot not found, unable to write transform points ', self.red_message)

            

            num_robot_points, num_total_points = 0, 0

            #handle the robot pose first
            if (robot_pos != None and total_pos != None):

                #check if the file has not been created
                if (not os.path.isfile(robot_pose_filename)):
                    with open(robot_pose_filename, 'w') as writer:
                        writer.write(str(1)+'\n') 
                        writer.write(str(robot_pos[0])+ ' '+str(robot_pos[1])+ ' '+str(robot_pos[2])+'\n') 

                else:
                    data = []
                    with open(robot_pose_filename, 'r') as f:
                        data = f.readlines()

                    data[0] = str(int(data[0]) + 1)+'\n' #increment the number of points
                    num_robot_points = int(data[0])+1
                    data.append(str(robot_pos[0])+ ' '+str(robot_pos[1])+ ' '+str(robot_pos[2])+'\n') #add the new point

                    with open(robot_pose_filename, 'w') as f:
                        f.writelines( data )

                self.printMessage("Saved robot pose.", self.green_message) #checked

            else:
                self.printMessage('Looks like nothing was ever published for the robot pos', self.red_message) #checked


            #handle the total pose next
            if (total_pos != None and robot_pos != None):

                #check if the file has not been created
                if (not os.path.isfile(total_pose_filename)):
                    with open(total_pose_filename, 'w') as writer:
                        writer.write(str(1)+'\n') 
                        writer.write(str(total_pos[0])+ ' '+str(total_pos[1])+ ' '+str(total_pos[2])+'\n') 

                else:
                    data = []
                    with open(total_pose_filename, 'r') as f:
                        data = f.readlines()

                    data[0] = str(int(data[0]) + 1)+'\n' #increment the number of points
                    num_total_points = int(data[0])+1
                    data.append(str(total_pos[0])+ ' '+str(total_pos[1])+ ' '+str(total_pos[2])+'\n') #add the new point

                    with open(total_pose_filename, 'w') as f:
                        f.writelines( data )

                self.printMessage("Saved total pose.", self.green_message) #checked

            else:
                self.printMessage('Looks like nothing was ever published for the total pos', self.red_message) #checked


            #ensure we are writitng the same number of points
            if (num_robot_points != num_total_points):
                self.printMessage('Error: The number of robot points does not equal the number of total station points', self.red_message)   


    def initControlPanel(self, pos):
        '''
        Initial the panel containing e-stop command, etc for each robot
        '''

        return #todo, delete this function

        #define the overall widget
        self.control_widget = QWidget()
        self.control_layout = qt.QGridLayout()
        self.control_buttons = []

        control_label = qt.QLabel()
        control_label.setText('CONTROL PANEL')
        control_label.setAlignment(Qt.AlignCenter)
        self.control_layout.addWidget(control_label)

        #define the number of commands in a single column
        num_in_col = 6

        #establish the sub-panel for each robot
        for robot_num, robot_name in enumerate(self.ros_gui_bridge.robot_names):
            #define the layout and group for a single robot
            robot_layout = qt.QGridLayout()
            robot_groupbox = qt.QGroupBox("Controls "+robot_name)

            #add the robot commands
            row, col = [0, 0] #the row and column to put the buttons
            robot_button_list = []
            for j, command in enumerate(self.ros_gui_bridge.robot_commands):
                if(row==num_in_col): #if we have filled this column, move to the next
                    row = 0
                    col+=1

                
                #check that we're not adding specific buttons to specific vehicles
                if  (not ((robot_name.find('erial') != -1) and (command=='Drop comms'))) and \
                   (not ((robot_name.find('erial') != -1) and (command=='Show bluetooth')))  : 

                    if (robot_name.find('erial') != -1): #change the text if its for the aerial vehicle
                        button = qt.QPushButton(self.ros_gui_bridge.remap_to_aerial_commands[command])
                    else:
                        button = qt.QPushButton(command)

                    
                    if (command == 'Resume'):
                        button.setCheckable(True) # a button pressed will stay pressed, until unclicked
                        button.setStyleSheet("QPushButton:checked { background-color: green }") #a button stays green when its in a clicked state

                    elif (command in ['Confirm', 'Cancel']):
                        button.setStyleSheet("background-color:rgb(126, 126, 126)")
                        button.setEnabled(False)

                    elif (command in ['Return home', 'Drop comms']):
                        button.setStyleSheet("QPushButton:pressed { background-color: red }") #a button stays red when its in a clicked state

                    elif (command == 'Hard e-stop') or \
                         ((robot_name.find('erial') != -1) and (command=='Soft e-stop')):

                         button.setCheckable(True)
                        
                    else:
                        button.setCheckable(True) # a button pressed will stay pressed, until unclicked
                        button.setStyleSheet("QPushButton:checked { background-color: red }") #a button stays red when its in a clicked state
                        

                    
                    robot_button_list.append(button)
                    
                    
                    #upon press, do something in ROS
                    button.clicked.connect(partial(self.processRobotCommandPress, command, robot_name, button))

                    robot_layout.addWidget(button, row, col)

                    row+=1


            #add a combobox to set the max run time for the aerial vehicle
            if (robot_name.find('erial') != -1):
                max_time_list = np.arange(0., 10.5, 0.5).tolist()
                max_time_box = qt.QComboBox()

                for speed in max_time_list:
                    max_time_box.addItem(str(speed))
                
                max_time_box.currentTextChanged.connect(partial(self.ros_gui_bridge.adjustMaxTime, robot_name, max_time_box)) 

                robot_layout.addWidget(max_time_box, row, col)


            #add a slider to set the max run time for the aerial vehicle
            # if (robot_name.find('erial') != -1):

            #     # slider_layout = qt.QGridLayout()
            #     slider_min, slider_max, slider_step = 0.5, 10, 0.5

            #     slider_label = qt.QLabel("Max flight time")
            #     slider_label.setAlignment(Qt.AlignCenter)
            #     robot_layout.addWidget(slider_label, num_in_col, 0, 1, 2)

            #     slider = qt.QSlider(Qt.Horizontal)
            #     slider.setMinimum(slider_min)
            #     slider.setMaximum(slider_max/slider_step)
            #     slider.setValue(slider_min)
            #     slider.setTickPosition(qt.QSlider.TicksBelow)
            #     slider.setTickInterval(slider_step)
            #     robot_layout.addWidget(slider, num_in_col+1, 0, 1, 2) #put it on the bottom

            #     #slider text
            #     slider_text = str(slider_min)+'    '
            #     i = slider_min + slider_step
            #     while i <= slider_max:
            #         if (int(i) == i):
            #             slider_text += str(int(i))+'     '
            #         else:
            #             slider_text += '     '
            #         i+=slider_step

            #     print slider_text

            #     tick_label = qt.QLabel(slider_text)
            #     tick_label.setAlignment(Qt.AlignCenter)
            #     robot_layout.addWidget(tick_label, num_in_col+2, 0, 1, 2)
                
                # slider.valueChanged.connect(self.valuechange)



            self.control_buttons.append(robot_button_list)



            #add the layout to a groupbox
            robot_groupbox.setLayout(robot_layout)

            self.control_layout.addWidget(robot_groupbox)

        #add the control panel to the overall gui
        self.control_widget.setLayout(self.control_layout)
        self.global_widget.addWidget(self.control_widget, pos[0], pos[1], pos[2], pos[3])

    def processRobotCommandPress(self, command, robot_name, button):
        '''
        Ensure that we dont press simulatenously activate
        conflicting command buttons (e.g. pause and soft estop)
        '''

        #if its an estop button, de-activate all other estop buttons
        if ((robot_name.find('ound') != -1) and (button.text() in self.ros_gui_bridge.estop_commands)) or\
             ((robot_name.find('erial') != -1) and (self.ros_gui_bridge.remap_from_aerial_commands[button.text()] in self.ros_gui_bridge.estop_commands)):

            #find the set of control buttons for this robot
            robot_ind = -1
            for i, name in enumerate(self.ros_gui_bridge.robot_names):
                if (robot_name == name):
                    robot_ind = i


            if (robot_ind!=-1):
                control_buttons = self.control_buttons[robot_ind]


                for cmd_button in control_buttons:
                    if (cmd_button != button) and  \
                            (((robot_name.find('ound') != -1) and cmd_button.text() in self.ros_gui_bridge.estop_commands) or \
                             ((robot_name.find('erial') != -1) and self.ros_gui_bridge.remap_from_aerial_commands[cmd_button.text()] in self.ros_gui_bridge.estop_commands)):
                        
                        cmd_button.setChecked(False)

        if (command == 'Confirm'):

            cmd_buttons = self.control_buttons[self.ros_gui_bridge.robot_names.index(robot_name)]

            for cmd_button in cmd_buttons:
                #de-activate the confirm/cancel buttons for that robot
                if (cmd_button.text() in ['Confirm', 'Cancel']): 
                    cmd_button.setEnabled(False)
                    cmd_button.setStyleSheet("background-color:rgb(126, 126, 126)")

                
                # if (cmd_button != button) and  \
                #         ((cmd_button.text() in self.ros_gui_bridge.estop_commands) or \
                #          (self.ros_gui_bridge.remap_from_aerial_commands[cmd_button.text()] in self.ros_gui_bridge.estop_commands)):
                    
                #     cmd_button.setChecked(False)

                # else:

                #find the estop button that was previously pressed
                if (cmd_button.isChecked()):
                    if (robot_name.find('erial') != -1):
                        cmd = self.ros_gui_bridge.remap_from_aerial_commands[cmd_button.text()]
                    else:
                        cmd = cmd_button.text()

                    if  (cmd in self.ros_gui_bridge.estop_commands):
                        cmd_button.setStyleSheet("QPushButton:checked { background-color: red }")
                        self.ros_gui_bridge.publishRobotCommand(cmd, robot_name, cmd_button)

                    else:#de-select the other estop buttons
                        cmd_button.setChecked(False)

            return

        elif (command == 'Cancel'):
            
            cmd_buttons = self.control_buttons[self.ros_gui_bridge.robot_names.index(robot_name)]

            for cmd_button in cmd_buttons:
                #de-activate the confirm/cancel buttons for that robot
                if (cmd_button.text() in ['Confirm', 'Cancel']): 
                    cmd_button.setEnabled(False)
                    cmd_button.setStyleSheet("background-color:rgb(126, 126, 126)")

                #de-select the estop button that was pressed
                if (robot_name.find('erial') != -1):
                    cmd = self.ros_gui_bridge.remap_from_aerial_commands[cmd_button.text()]
                else:
                    cmd = cmd_button.text()

                if (cmd in self.ros_gui_bridge.estop_commands):
                    cmd_button.setChecked(False)

            return




        #if its a button we need confirmation about, let's get confirmation
        elif button.isChecked() and \
             ((command =='Hard e-stop') or \
                (robot_name.find('erial') != -1 and self.ros_gui_bridge.remap_to_aerial_commands[command] == 'Land')):

            #activate the confirm/cancel buttons for that robot
            cmd_buttons = self.control_buttons[self.ros_gui_bridge.robot_names.index(robot_name)]

            for cmd_button in cmd_buttons:
                if (cmd_button.text() =='Confirm'):
                    cmd_button.setEnabled(True)
                    cmd_button.setStyleSheet("background-color:rgb(0, 220, 0)")

                elif(cmd_button.text() =='Cancel'):
                    cmd_button.setEnabled(True)
                    cmd_button.setStyleSheet("background-color:rgb(220, 0, 0)")

            return



        


        #de-activate the appropriate buttons
        cmd_buttons = self.control_buttons[self.ros_gui_bridge.robot_names.index(robot_name)]

        for cmd_button in cmd_buttons:
            #de-activate the confirm/cancel buttons for that robot
            if (cmd_button.text() in ['Confirm', 'Cancel']): 
                cmd_button.setEnabled(False)
                cmd_button.setStyleSheet("background-color:rgb(126, 126, 126)")

        self.ros_gui_bridge.publishRobotCommand(command, robot_name, button)





    def initArtifactVisualizer(self, pos):
        '''
        Panel to visualize the artifacts coming in
        '''

        #define the overall widget
        self.artvis_widget = QWidget()
        self.artvis_layout = qt.QGridLayout()

        #exapnd to fill the space vertically
        art_label = qt.QLabel()
        art_label.setText('ARTIFACT VISUALIZATION')
        art_label.setAlignment(Qt.AlignCenter)
        self.artvis_layout.addWidget(art_label, 0, 0, 1, 2)


        # self.bluetooth_indicator_label = qt.QLabel()
        # self.bluetooth_indicator_label.setText('Bluetooth: Good (Fake)')
        # self.bluetooth_indicator_label.setAlignment(Qt.AlignCenter)
        # self.artvis_layout.addWidget(self.bluetooth_indicator_label, 1,0 )

        # self.audio_indicator_label = qt.QLabel()
        # self.audio_indicator_label.setText('Audio: Good (Fake)')
        # self.audio_indicator_label.setAlignment(Qt.AlignCenter)
        # self.artvis_layout.addWidget(self.audio_indicator_label, 1,1 )  



        #add in a blank label to represent an object
        self.art_image = qt.QLabel()
        
        rospack = rospkg.RosPack()
        img_filename = rospack.get_path('basestation_gui_python')+'/src/black_img.png'

        img = cv2.imread(img_filename)
        img = cv2.resize(img,(self.artifact_img_width, self.artifact_img_length))

        img_height, img_width = img.shape[:2]
        img = gui.QImage(img, img_width, img_height, gui.QImage.Format_RGB888)
        img = gui.QPixmap.fromImage(img)
        self.art_image.setPixmap(img)
        self.art_image.mousePressEvent = self.ros_gui_bridge.publishImageCoord
        # self.art_image.setAlignment(Qt.AlignCenter)

        self.artvis_layout.addWidget(self.art_image, 1, 0, 3, 2) #last 2 parameters are rowspan and columnspan

        self.update_art_label = qt.QLabel()
        self.update_art_label.setText('Updates here')
        self.update_art_label.setAlignment(Qt.AlignCenter)
        self.update_art_label.setStyleSheet("background-color: rgba(126, 126, 126,50%)")
        self.update_art_label.hide()
        self.artvis_layout.addWidget(self.update_art_label, 3, 0, 1, 2)
        

        #add to the overall gui
        self.artvis_widget.setLayout(self.artvis_layout)
        self.global_widget.addWidget(self.artvis_widget, pos[0], pos[1], pos[2], pos[3])

    def displayArtifactImg(self, artifact, index = 0):
        '''
        Display an artifact's index-th image
        '''

        # print "disp art image:",isinstance(threading.current_thread(), threading._MainThread)
        # print "disp art image:",threading.current_thread().__class__.__name__



        if (index+1) > len(artifact.imgs):
            rospack = rospkg.RosPack()
            img_filename = rospack.get_path('basestation_gui_python')+'/src/black_img.png'

            img = cv2.imread(img_filename)
       
        else:
            img = artifact.imgs[index]
        
        img = cv2.resize(img,(self.artifact_img_width, self.artifact_img_length))

        img_height, img_width = img.shape[:2]
        img = gui.QImage(img, img_width, img_height, gui.QImage.Format_RGB888)
        img = gui.QPixmap.fromImage(img)
        self.art_image.setPixmap(img)
        # self.art_image.setAlignment(Qt.AlignCenter)


        self.artifact_image_index = index

        #change the label
        if ((index+1) > len(artifact.imgs)) or (len(artifact.imgs) == 0):
            self.img_displayed_label.setText('Img 0/0')
        else:
            self.img_displayed_label.setText('Img '+str(index+1)+'/'+str(len(artifact.imgs)))

    def processArtRefinementPress(self):
        '''
        Process pressing of the button, which visualizes the 
        interactive marker for artifact refinement
        '''
        if (self.displayed_artifact != None):
            self.ros_gui_bridge.publishRefinementMarkerPos(self.displayed_artifact, self.art_refinement_button)
        else:
            self.printMessage("Request not processed. No artifact currently being displayed", self.normal_message)
        
    def updateRefinmentPos(self, msg):
        self.update_refinement_pos_trigger.emit(msg)    

    def updateRefinmentPosMonitor(self, msg):
        '''
        When the interactive marker moves, this 
        is called to update the appropriate textboxes and 
        update the artifact position
        '''

        # print "update refinement pos:",isinstance(threading.current_thread(), threading._MainThread)
        # print "update refinement pos:",threading.current_thread().__class__.__name__

        #change the textboxes
        self.art_pos_textbox_x.setText(str(msg.pose.position.x)[:7])
        self.art_pos_textbox_y.setText(str(msg.pose.position.y)[:7])
        self.art_pos_textbox_z.setText(str(msg.pose.position.z)[:7])

        #change the artifact 
        if(self.displayed_artifact != None):
            self.displayed_artifact.pos[0] = msg.pose.position.x
            self.displayed_artifact.pos[1] = msg.pose.position.y
            self.displayed_artifact.pos[2] = msg.pose.position.z

    def imgBack(self):
        '''
        Display the previous image for an artifact
        '''

        if(self.displayed_artifact != None):

            if (len(self.displayed_artifact.imgs) == 0):
                self.printMessage("There are no images to show", self.normal_message) #checked

            else:
                #check if we cannot go back any farther
                if (self.artifact_image_index == 0):
                    self.displayArtifactImg(self.displayed_artifact, len(self.displayed_artifact.imgs) - 1)

                #else, just go back normally
                else:
                    self.displayArtifactImg(self.displayed_artifact, self.artifact_image_index - 1)
        
        else:
            self.printMessage("No artifact currently being displayed. Select one from the queue.", self.normal_message)

    def imgForward(self):
        '''
        Display the next image for an artifact
        '''
        if(self.displayed_artifact != None):

            if (len(self.displayed_artifact.imgs) == 0):
                self.printMessage("There are no images to show", self.normal_message) #checked

            else:
                #check if we cannot go forward any farther
                if (self.artifact_image_index == (len(self.displayed_artifact.imgs) - 1)):
                    self.displayArtifactImg(self.displayed_artifact,  0)

                #else, just go forward normally
                else:
                    self.displayArtifactImg(self.displayed_artifact, self.artifact_image_index + 1)
        
        else:
            self.printMessage("No artifact currently being displayed. Select one from the queue.", self.normal_message)

    def initArtifactManipulator(self, pos):
        '''
        Buttons and textboxes to refine the position and complete actions with the  artifact
        '''

         #define the overall widget
        self.artmanip_widget = QWidget()
        self.artmanip_layout = qt.QGridLayout() 

        boldFont = gui.QFont()
        boldFont.setBold(True) 

        #add arrow buttons and label in the middle to indicate what image we're on
        self.img_back_button = qt.QPushButton("<-")
        self.img_back_button.clicked.connect(self.imgBack)
        self.artmanip_layout.addWidget(self.img_back_button, 0, 0 )

        self.img_displayed_label = qt.QLabel()
        self.img_displayed_label.setText('Img 0/0')
        self.img_displayed_label.setFont(gui.QFont(None, 16, gui.QFont.Bold) )
        self.img_displayed_label.setAlignment(Qt.AlignCenter)
        self.artmanip_layout.addWidget(self.img_displayed_label, 0,1)

        self.img_forward_button = qt.QPushButton("->")
        self.img_forward_button.clicked.connect(self.imgForward)
        self.artmanip_layout.addWidget(self.img_forward_button, 0, 2 )

              


        #add in information about 3d position        

        #information about the detected position
        robot_pos = ['N/A', 'N/A', 'N/A'] #fake data

        self.orig_pos_label = qt.QLabel()
        self.orig_pos_label.setText('\n\nOriginal Position (XYZ)')
        self.orig_pos_label.setAlignment(Qt.AlignCenter)
        self.orig_pos_label.setFont(boldFont)
        self.artmanip_layout.addWidget(self.orig_pos_label, 1, 0, 1, 3)

        self.orig_pos_label_x = qt.QLabel()
        self.orig_pos_label_x.setText(str(robot_pos[0]))
        self.orig_pos_label_x.setAlignment(Qt.AlignCenter)
        self.artmanip_layout.addWidget(self.orig_pos_label_x, 2, 0)

        self.orig_pos_label_y = qt.QLabel()
        self.orig_pos_label_y.setText(str(robot_pos[1]))
        self.orig_pos_label_y.setAlignment(Qt.AlignCenter)
        self.artmanip_layout.addWidget(self.orig_pos_label_y, 2, 1)

        self.orig_pos_label_z = qt.QLabel()
        self.orig_pos_label_z.setText(str(robot_pos[2]))
        self.orig_pos_label_z.setAlignment(Qt.AlignCenter)
        self.artmanip_layout.addWidget(self.orig_pos_label_z, 2, 2)

        #editable information about the position, to send to darpa
        self.refined_pos_label = qt.QLabel()
        self.refined_pos_label.setText('\nRefined Position (XYZ)')
        self.refined_pos_label.setFont(boldFont)
        self.refined_pos_label.setAlignment(Qt.AlignCenter)
        self.artmanip_layout.addWidget(self.refined_pos_label, 3, 0, 1, 3)

        self.art_pos_textbox_x, self.art_pos_textbox_y, self.art_pos_textbox_z = qt.QLineEdit(), qt.QLineEdit(), qt.QLineEdit()
       
        #fill in some fake data
        self.art_pos_textbox_x.setText(str(robot_pos[0]))
        self.artmanip_layout.addWidget(self.art_pos_textbox_x, 4, 0)

        self.art_pos_textbox_y.setText(str(robot_pos[1]))
        self.artmanip_layout.addWidget(self.art_pos_textbox_y, 4, 1)

        self.art_pos_textbox_z.setText(str(robot_pos[2]))
        self.artmanip_layout.addWidget(self.art_pos_textbox_z, 4, 2)

        #add button for displaying the interactive marker
        self.art_refinement_button = qt.QPushButton("Show Refinement Marker")
        self.art_refinement_button.setCheckable(True)
        self.art_refinement_button.clicked.connect(self.processArtRefinementPress)
        self.artmanip_layout.addWidget(self.art_refinement_button, 5, 0 , 1, 3)



 #add in a few buttons at the bottom to do various things
        self.artmanip_button_layout = qt.QHBoxLayout()
        self.artmanip_button_list = []

        #add the buttons and textboxes at the bottom
        art_action_label1 = qt.QLabel()
        # art_action_label1.setText('\n\nAction')
        # art_action_label1.setAlignment(Qt.AlignCenter)
        # self.artmanip_layout.addWidget(art_action_label1, 3, 0, 1, 2)

        art_action_label2 = qt.QLabel()
        art_action_label2.setText('\n\nCategory')
        art_action_label2.setFont(boldFont)
        art_action_label2.setAlignment(Qt.AlignCenter)
        self.artmanip_layout.addWidget(art_action_label2, 6, 0, 1, 2)

        art_action_label3 = qt.QLabel()
        art_action_label3.setText('\n\nPriority')
        art_action_label3.setFont(boldFont)
        art_action_label3.setAlignment(Qt.AlignCenter)
        self.artmanip_layout.addWidget(art_action_label3, 6, 2)

         #make the combobox for setting the artifact category
        self.darpa_cat_box = qt.QComboBox()
        
        for category in self.ros_gui_bridge.artifact_categories:
            self.darpa_cat_box.addItem(category)
        
        self.darpa_cat_box.currentTextChanged.connect(self.updateArtifactCat)

        self.artmanip_layout.addWidget(self.darpa_cat_box, 7, 0, 1, 2)

        #make the combobox for setting the artifact priority
        self.artifact_priority_box = qt.QComboBox() 

        self.artifact_priority_box.addItem('High')
        self.artifact_priority_box.addItem('Med')
        self.artifact_priority_box.addItem('Low')
        
        self.artifact_priority_box.currentTextChanged.connect(self.updateArtifactPriority)
        self.artmanip_layout.addWidget(self.artifact_priority_box, 7, 2)


        self.darpa_button = qt.QPushButton("To DARPA")
        self.darpa_button.clicked.connect(partial(self.decideArtifact))
        self.darpa_button.setStyleSheet("background-color:rgb(255,130,0)")
        self.artmanip_layout.addWidget(self.darpa_button, 8, 0, 1, 3)

        self.darpa_confirm_button = qt.QPushButton("Confirm")
        self.darpa_confirm_button.clicked.connect(partial(self.proposeArtifact))
        self.darpa_confirm_button.setStyleSheet("background-color:rgb(100, 100, 100)")
        self.darpa_confirm_button.setEnabled(False)
        self.artmanip_layout.addWidget(self.darpa_confirm_button, 9, 2)

        self.darpa_cancel_button = qt.QPushButton("Cancel")
        self.darpa_cancel_button.clicked.connect(partial(self.cancelProposal))
        self.darpa_cancel_button.setStyleSheet("background-color:rgb(100, 100, 100)")
        self.darpa_cancel_button.setEnabled(False)
        self.artmanip_layout.addWidget(self.darpa_cancel_button, 9, 0)


            

        #add to the overall gui
        self.artmanip_widget.setLayout(self.artmanip_layout)
        self.global_widget.addWidget(self.artmanip_widget, pos[0], pos[1], pos[2], pos[3])    

    

    def proposeArtifact(self):
        thread = threading.Thread(target=self.proposeArtifactThread)
        thread.start()

    def proposeArtifactThread(self):
        '''
        Function for proposing an artifact to darpa and then changing gui components correspondingly
        '''
        if (self.art_pos_textbox_x.text()=='') or (self.art_pos_textbox_y.text()=='') or (self.art_pos_textbox_z.text()==''):
            self.printMessage("Nothing proposed. Please select an artifact from the queue", self.red_message)

        elif (self.displayed_artifact==None):
            self.printMessage("Nothing proposed. Please select an artifact from the queue", self.red_message)

        elif(self.connect_to_command_post):

            #transform the points into the darpa frame
            point = np.array([ float(self.art_pos_textbox_x.text()), float(self.art_pos_textbox_y.text()), \
                         float(self.art_pos_textbox_z.text()) ])

            #transform based on which robot submitted the detection
            if (self.displayed_artifact != None):
                transformed_point = self.toDarpaFrame(point, self.displayed_artifact.source_robot)
                data = [float(transformed_point[0]), float(transformed_point[1]), float(transformed_point[2]), self.darpa_cat_box.currentText()]
                proposal_return = self.darpa_gui_bridge.startArtifactProposal(data)

                if(len(proposal_return) > 0):
                    self.drawAfterProposal(proposal_return)
            
            
        
        else:
            self.printMessage('Not connected to DARPA basestation, thus artifact not submitted.', self.red_message)

    def drawAfterProposal(self, proposal_return):
        self.arthist_trigger.emit(proposal_return)

    def drawAfterProposalMonitor(self, proposal_return):
        # print "drawing in arthist:",isinstance(threading.current_thread(), threading._MainThread)
        # print "drawing in arthist:",threading.current_thread().__class__.__name__

        with self.artifact_proposal_lock: #to ensure we only draw one response at once

            self.printMessage("Submitted artifact of category: " + self.darpa_cat_box.currentText(), self.green_message) #checked
            
            [submission_time_raw, artifact_type, x, y, z, report_status, score_change, http_response, http_reason] = \
                                                                                            proposal_return



            #publish the message to display in thesubmission reply plugin
            msg = ArtifactSubmissionReply()
            msg.submission_time_raw = float(submission_time_raw)
            msg.artifact_type = str(artifact_type)
            msg.x = x
            msg.y = y
            msg.z = z
            msg.report_status = str(report_status)
            msg.score_change = score_change
            msg.http_response = str(http_response)
            msg.http_reason = str(http_reason)

            self.submission_reply_pub.publish(msg)

            

            self.printMessage('DARPA response: '+str(report_status)+' HTTP Response: '+str(http_response)+str(http_reason)+\
                                     ' Submission Correct? '+str((score_change!=0)), self.green_message) #checked

            

            #update the artifact's darpa_response property
            self.displayed_artifact.darpa_response = 'DARPA response: '+str(report_status)+'\nHTTP Response: '+\
                                                     str(http_response)+str(http_reason)+\
                                                     '\nSubmission Correct? '+str((score_change!=0))

            #update the artifacts' to_darpa time
            try:
                self.displayed_artifact.time_to_darpa = int(float(submission_time_raw))
            except:
                self.displayed_artifact.time_to_darpa =  9999



            #go find the artifact in the queue and remove it
            with self.update_queue_lock:
                self.queue_table.setSortingEnabled(False)
                row_ind = self.findDisplayedArtifact()

                if(row_ind!=-1):
                    self.queue_table.removeRow(self.queue_table.item(row_ind,0).row())


                    #also remove it from the gui engine artifacts list and put it into the gui engine proposed list
                    if self.displayed_artifact in self.gui_engine.queued_artifacts:
                        self.gui_engine.queued_artifacts.remove(self.displayed_artifact)

                    self.gui_engine.submitted_artifacts.append(self.displayed_artifact)

                self.queue_table.setSortingEnabled(True)


            #remove the artifact from the main visualization panel

            #display a blank box for the image
            self.displayArtifactImg(self.displayed_artifact, len(self.displayed_artifact.imgs))

            self.orig_pos_label_x.setText('')
            self.art_pos_textbox_x.setText('')

            self.orig_pos_label_y.setText('')
            self.art_pos_textbox_y.setText('')

            self.orig_pos_label_z.setText('')
            self.art_pos_textbox_z.setText('')

            self.displayed_artifact = None

            #remove any update on the artifact (messages about it being deleted or updated)
            self.update_art_label.hide()

        #reset the darpa proposal buttons
        self.darpa_confirm_button.setEnabled(False)
        self.darpa_cancel_button.setEnabled(False)

        self.darpa_confirm_button.setStyleSheet("background-color:rgb(126, 126, 126)")
        self.darpa_cancel_button.setStyleSheet("background-color:rgb(126, 126, 126)")

        self.gui_engine.savePeriodically(self)

    def toDarpaFrame(self, point, robot_num):
        '''
        Function to convert an artifact detection location into
        the darpa frame
        '''

        point = np.float32([point[0], point[1], point[2], 1])

        if (robot_num >= 0 and robot_num < self.ros_gui_bridge.robot_names \
                         and self.ros_gui_bridge.robot_names[robot_num].find('round') != -1) or (robot_num == -1):
            
            self.printMessage('Transform wrt ground', self.normal_message) #checked
            return np.matmul(self.darpa_transform_ugv, point)

        elif (robot_num >= 0 and robot_num < self.ros_gui_bridge.robot_names \
                         and self.ros_gui_bridge.robot_names[robot_num].find('eria') != -1) or (robot_num == -2):
            
            self.printMessage('Transform wrt aerial', self.normal_message) #checked
            return np.matmul(self.darpa_transform_uav, point)

        else:
            self.printMessage('Could not find robot transform, artifact not transformed', self.red_message)
            return point



    def decideArtifact(self):
        '''
        Initializes the sequence for final decision on whether we want to propose an artifact
        or not
        '''

        if (self.art_pos_textbox_x.text()=='') or (self.art_pos_textbox_y.text()=='') or (self.art_pos_textbox_z.text()==''):
            self.printMessage("Nothing proposed. Please select an artifact from the queue", self.normal_message) #checked

        elif (self.displayed_artifact==None):
            self.printMessage("Nothing proposed. Please select an artifact from the queue", self.normal_message) #checked

        elif(self.connect_to_command_post):

            #enable the confirm and cancel buttons
            self.darpa_confirm_button.setEnabled(True)
            self.darpa_cancel_button.setEnabled(True) 

            #color the buttons appropriately
            self.darpa_confirm_button.setStyleSheet("background-color:rgb(0,220,0)")
            self.darpa_cancel_button.setStyleSheet("background-color:rgb(220,0,0)")

        else:
            self.printMessage('Not connected to DARPA basestation, thus artifact not submitted.', self.red_message)
        


    def cancelProposal(self):
        '''
        We started a proposal process but decided against it
        '''

        #reset the darpa proposal buttons
        self.darpa_confirm_button.setEnabled(False)
        self.darpa_cancel_button.setEnabled(False)

        self.darpa_confirm_button.setStyleSheet("background-color:rgb(126, 126, 126)")
        self.darpa_cancel_button.setStyleSheet("background-color:rgb(126, 126, 126)")
        
                
    def findDisplayedArtifact(self):
        '''
        Returns the row of the artifact table for the displayed artifact
        '''

        if (self.queue_table.isSortingEnabled()):
            self.printMessage("Table did not have sorting disabled!! (Debugging message)", self.normal_message)
            self.queue_table.setSortingEnabled(False)

        if(self.displayed_artifact!=None): #if we have actually set the artifact
            for i in range(self.queue_table.rowCount()):

                if (self.displayed_artifact.unique_id == self.queue_table.item(i, 5).text()):
                    return i

        self.printMessage("Could not find displayed artifact. Please select an artifact" , self.red_message)
        return -1


    def dispToSeconds(self, disp_seconds):
        '''
        Convert from displayed seconds to actual seconds
        '''
        colon = disp_seconds.find(':')
        from_minutes = float(disp_seconds[:colon])*60
        from_seconds = float(disp_seconds[colon+1:])

        return int(from_minutes + from_seconds)



    def updateDisplayedArtifact(self, artifact):
        '''
        Update the info for the artifact being displayed
        '''
        self.displayed_artifact = artifact

    def getDisplayedArtifact(self):
        '''
        Get the info for the artifact being displayed
        '''
        return self.displayed_artifact

    # def updateArtifactPriority(self):
    #     self.update_artifact_priority_trigger.emit()

    def updateArtifactPriority(self):
        '''
        The combo box for changing the artifact priority was pressed
        self.dont_change_art_priority indicates if the change is automatic after clicking 
        on an artifact in the  queue and therefore don't do anything
        '''

        # print "priority: ",isinstance(threading.current_thread(), threading._MainThread)
        # print "priority: ",threading.current_thread().__class__.__name__

        if (not self.dont_change_art_priority) and self.displayed_artifact!=None :

            with self.update_queue_lock:
                
                #change the text in the queue
                self.queue_table.setSortingEnabled(False)
                
                row_ind = self.findDisplayedArtifact()

                if(row_ind!=-1):                
                    self.queue_table.setItem(row_ind, 1, qt.QTableWidgetItem(self.artifact_priority_box.currentText()))
                    self.queue_table.item(row_ind, 1).setTextAlignment(Qt.AlignHCenter) 
                    self.queue_table.item(row_ind, 1).setFlags( core.Qt.ItemIsSelectable |  core.Qt.ItemIsEnabled )

                    self.displayed_artifact.priority = self.artifact_priority_box.currentText()

                self.queue_table.setSortingEnabled(True)

                self.queue_table.viewport().update()

        self.dont_change_art_priority = False #reset its value

    # def updateArtifactCat(self):
    #     self.update_artifact_category_trigger.emit()

    def updateArtifactCat(self):
        '''
        The combo box for changing the artifact category was pressed
        '''

        #first check the combobox change came from a  human and not a automatic call
        if (not self.dont_change_art_category) and self.displayed_artifact!=None :    
       

            #change the text in the queue
            with self.update_queue_lock:
                self.queue_table.setSortingEnabled(False)
                
                row_ind = self.findDisplayedArtifact()
                
                if(row_ind!=-1):
                    self.queue_table.setItem(row_ind, 3, qt.QTableWidgetItem(self.darpa_cat_box.currentText()))
                    self.queue_table.item(row_ind, 3).setTextAlignment(Qt.AlignHCenter) 
                    self.queue_table.item(row_ind, 3).setFlags( core.Qt.ItemIsSelectable |  core.Qt.ItemIsEnabled )

                    self.displayed_artifact.category = self.darpa_cat_box.currentText()

                self.queue_table.setSortingEnabled(True)

                self.queue_table.viewport().update()

        self.dont_change_art_category = False #reset its value
        

    def sendToQueue(self,artifact):
        self.queue_trigger.emit(artifact)

    def sendToQueueMonitor(self, artifact):
        '''
        Send the artifact subscribed to, to the queue
        '''

        # print "queue table:",isinstance(threading.current_thread(), threading._MainThread)
        # print "queue table:",threading.current_thread().__class__.__name__


        with self.update_queue_lock:

            #if we need to connect to the command post, wait
            #TODO: figure out why we would wait to be connected to the command post
            if (self.connect_to_command_post and self.darpa_gui_bridge.darpa_status_update['run_clock']==None):

                #remove the artifact
                if artifact in self.gui_engine.queued_artifacts:
                    self.gui_engine.queued_artifacts.remove(artifact)

                if artifact in self.gui_engine.all_artifacts:
                    self.gui_engine.all_artifacts.remove(artifact)

                return False

            else:

                self.queue_table.setSortingEnabled(False) #to avoid corrupting the table

                self.queue_table.insertRow(self.queue_table.rowCount())
                row = self.queue_table.rowCount() - 1


                #fill in the row data
                if (artifact.time_from_robot == -1 and self.connect_to_command_post and self.darpa_gui_bridge.darpa_status_update['run_clock']!=None): #this is coming directly from the robot
                    try:
                        disp_time = self.darpa_gui_bridge.displaySeconds(float(self.darpa_gui_bridge.darpa_status_update['run_clock']))
                    except:
                        disp_time = '99:99'
                
                elif(self.connect_to_command_post):
                    disp_time = self.darpa_gui_bridge.displaySeconds(float(artifact.time_from_robot))

                else:
                    disp_time = self.darpa_gui_bridge.displaySeconds(float(1))


                row_data = [artifact.source_robot, artifact.priority, disp_time, \
                            artifact.category, '!', artifact.unique_id]


                for col, val in enumerate(row_data):

                    if (col==2):
                        colon = disp_time.find(':')
                        val = float(disp_time[:colon])*60 + float(disp_time[colon+1:])
                        item = NumericItem(str(disp_time))
                        item.setData(core.Qt.UserRole, val)
                    
                    else: #if we're not dealing with a display time
                        item = NumericItem(str(val))
                        item.setData(core.Qt.UserRole, val)                    

                    self.queue_table.setItem(row, col, item)

                if(self.connect_to_command_post and self.darpa_gui_bridge.darpa_status_update['run_clock']!=None):
                    try:
                        artifact.time_from_robot = int(float(self.darpa_gui_bridge.darpa_status_update['run_clock']))
                    except:
                        artifact.time_from_robot = 9999

                else:
                    artifact.time_from_robot = 1

                #color the unread green
                self.queue_table.item(row, 4).setBackground(gui.QColor(0,255,0))

                for i in range(self.queue_table.columnCount()): #make the cells not editable and make the text centered
                    if self.queue_table.item(row, i) != None: 
                        self.queue_table.item(row, i).setFlags( core.Qt.ItemIsSelectable |  core.Qt.ItemIsEnabled )
                        self.queue_table.item(row, i).setTextAlignment(Qt.AlignHCenter) 

                self.queue_table.setSortingEnabled(True) #to avoid corrupting the table

                if(self.queue_table_sort_button.isChecked()): #if the sort button is pressed, sort the incoming artifacts
                    self.queue_table.sortItems(2, core.Qt.DescendingOrder)

                    self.queue_table.viewport().update()


   

    def initStatusPanel(self, pos):
        '''
        Panel to display information (battery health, comms signal, etc.)
        for each robot
        '''

        #define the overall widget
        self.status_widget = QWidget()
        self.status_layout = qt.QGridLayout()

        #preliminaries to build the rest of the panel
        num_robots = len(self.ros_gui_bridge.robot_names) #get the number of robots
        statuses = ['Battery(mins)', 'Comms', 'Mobility', 'CPU', 'Disk Space'] #define the status each robot will have
        self.statuses = statuses

        status_label = qt.QLabel()
        status_label.setText('STATUS PANEL')
        status_label.setAlignment(Qt.AlignCenter)
        self.status_layout.addWidget(status_label, 0,0)

        #make a table
        self.status_table = qt.QTableWidget()

        #resize the cells to fill the widget 
        self.status_table.horizontalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)
        self.status_table.verticalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)
        
        self.status_table.setRowCount(len(statuses)) # set row count
        self.status_table.setColumnCount(num_robots) # set column count

        #make the row and column headers
        self.status_table.setVerticalHeaderLabels(statuses) 
        self.status_table.setHorizontalHeaderLabels(self.ros_gui_bridge.robot_names) 

        

        #add the table to the layout
        self.status_layout.addWidget(self.status_table, 1, 0)

        #add to the overall gui
        self.status_widget.setLayout(self.status_layout)
        self.global_widget.addWidget(self.status_widget, pos[0], pos[1], pos[2], pos[3])

    def queueClick(self, row, col):
        '''
        Callback that is run when something in the artifact
        queue is selected
        '''

        # print "queue table click:",isinstance(threading.current_thread(), threading._MainThread)
        # print "queue table click:",threading.current_thread().__class__.__name__

        #remove any update on the artifact (messages about it being deleted or updated)
        self.update_art_label.hide()

        #disable darpa proposal buttons
        self.cancelProposal()

        #change show refinement marker 
        if (self.art_refinement_button.isChecked()):
            self.art_refinement_button.click()
        

        #remove the "unread" indicator if its there
        with self.update_queue_lock:
            self.queue_table.setItem(row,4, qt.QTableWidgetItem(str('')))
            self.queue_table.item(row, 4)#.setBackground(gui.QColor(0,255,0))#color the unread green

            self.queue_table.item(row, 4).setFlags( core.Qt.ItemIsSelectable |  core.Qt.ItemIsEnabled )
            self.queue_table.item(row, 4).setTextAlignment(Qt.AlignHCenter)

            clicked_robot_id =  int(self.queue_table.item(row, 0).text())
            priority = self.queue_table.item(row, 1).text()
            category = self.queue_table.item(row, 3).text()
            clicked_unique_id  = self.queue_table.item(row, 5).text()

            #highlight the entire row
            self.queue_table.selectRow(row)

        #save the refined position of the displayed artifact
        if(self.displayed_artifact != None):

            if (len(self.art_pos_textbox_x.text()) > 0):
                self.displayed_artifact.pos[0] = float(self.art_pos_textbox_x.text())

            if (len(self.art_pos_textbox_y.text()) > 0):
                self.displayed_artifact.pos[1] = float(self.art_pos_textbox_y.text())

            if (len(self.art_pos_textbox_z.text()) > 0):
                self.displayed_artifact.pos[2] = float(self.art_pos_textbox_z.text())  




        #change the combo boxes
        index = self.darpa_cat_box.findText(category, core.Qt.MatchFixedString)
        if index >= 0:
            self.dont_change_art_category = True #just change the gui, nothing else
            self.darpa_cat_box.setCurrentIndex(index)


        ind = self.artifact_priority_box.findText(priority, core.Qt.MatchFixedString)
        if ind >= 0:
            self.dont_change_art_priority = True  #just change the gui, nothing else
            self.artifact_priority_box.setCurrentIndex(ind)  




        #associate with a specific artifact
        #todo: put in some error checking
        for art in self.gui_engine.queued_artifacts:

            if ( art.unique_id == clicked_unique_id):
                
                artifact = art

                #change th original location of the artifact to be the current position
                artifact.orig_pos = artifact.pos

                #change the text of the xyz positions
                robot_pos = artifact.pos
                orig_robot_pos = artifact.orig_pos

                #fill in the positional data
                self.orig_pos_label_x.setText(str(orig_robot_pos[0])[:7])
                self.orig_pos_label_y.setText(str(orig_robot_pos[1])[:7])
                self.orig_pos_label_z.setText(str(orig_robot_pos[2])[:7])
                
                self.art_pos_textbox_x.setText(str(robot_pos[0])[:7])
                self.art_pos_textbox_y.setText(str(robot_pos[1])[:7])
                self.art_pos_textbox_z.setText(str(robot_pos[2])[:7])


                #update the global info for what artifact is being displayed
                self.updateDisplayedArtifact(artifact)

                #update the image being shown
                self.displayArtifactImg(artifact, 0)

                if(self.displayed_artifact!=None):
                    self.displayed_artifact.unread = False

                break

        self.dont_change_art_category = False  
        self.dont_change_art_priority = False

        

    def updateArtifactInQueue(self, artifact):
        self.update_artifact_trigger.emit(artifact)

    def updateArtifactInQueueMonitor(self, artifact):
        '''
        An artifact's info has been changed and the queue needs to be updated
        '''

        # print "update artifact:",isinstance(threading.current_thread(), threading._MainThread)
        # print "update artifact:",threading.current_thread().__class__.__name__

        for i in range(self.queue_table.rowCount()):

            if artifact.unique_id == self.queue_table.item(i, 5).text():

                row = i  

                #update the necessary info
                if (artifact.time_from_robot == -1): #this is coming directly from the robot
                    disp_time = self.darpa_gui_bridge.displaySeconds(float(self.darpa_gui_bridge.darpa_status_update['run_clock']))
                
                else:
                    disp_time = self.darpa_gui_bridge.displaySeconds(float(artifact.time_from_robot))

                update_msg = 'Updtd'

                if (self.queue_table.item(i, 4).text().find('!') != -1): #the messagehas not been read yet
                    update_msg = '!'

                row_data = [artifact.source_robot, artifact.priority, disp_time, \
                            artifact.category, update_msg, artifact.unique_id]


                with self. update_queue_lock:

                    self.queue_table.setSortingEnabled(False) #to avoid corrupting the table

                    for col, val in enumerate(row_data):

                        if (col == 2):
                            colon = disp_time.find(':')
                            val = float(disp_time[:colon])*60 + float(disp_time[colon+1:])
                            item = NumericItem(str(disp_time))
                            item.setData(core.Qt.UserRole, val)
                        
                        else:
                            item = NumericItem(str(val))
                            item.setData(core.Qt.UserRole, val)                          
                        
                        

                        self.queue_table.setItem(row, col, item)
                        # self.queue_table.itemChanged(item)#self.queue_table.item(row, col))


                    #color the unread green
                    self.queue_table.item(row, 4).setBackground(gui.QColor(0,255,0))

                    for k in range(self.queue_table.columnCount()): #make the cells not editable and make the text centered
                        if self.queue_table.item(row, k) != None: 
                            self.queue_table.item(row, k).setFlags( core.Qt.ItemIsSelectable |  core.Qt.ItemIsEnabled )
                            self.queue_table.item(row, k).setTextAlignment(Qt.AlignHCenter) 

                    #refresh the table
                    self.queue_table.viewport().update()

                    self.queue_table.setSortingEnabled(True) #to avoid corrupting the table

                    if(self.queue_table_sort_button.isChecked()): #if the sort button is pressed, sort the incoming artifacts
                        self.queue_table.sortItems(2, core.Qt.DescendingOrder)

                        self.queue_table.viewport().update()

        #if the artifact is currently displayed, bring up a textbox alerting them
        if (self.displayed_artifact != None) and (artifact.unique_id == self.displayed_artifact.unique_id):
            self.update_art_label.setText('Artifact has been updated. Please select it again in the queue')
            self.update_art_label.setStyleSheet("background-color: rgba(255, 255, 0, 80%)")
            self.update_art_label.show()
        
    def alertImgAboutRemoval(self):
        self.alert_about_art_removal_trigger.emit()


    def alertImgAboutRemovalMonitor(self):
        '''
        If the artifact is removed while the BSM is viewing it, pop up
        a message saying so
        '''

        # print "alert about removal:",isinstance(threading.current_thread(), threading._MainThread)
        # print "alert about removal:",threading.current_thread().__class__.__name__

        #if the artifact is currently displayed, bring up a textbox alerting them
        self.update_art_label.setText('Artifact has been deleted!')
        self.update_art_label.setStyleSheet("background-color: rgba(220, 0, 0, 70%)")
        self.update_art_label.show()

    def removeQueueItem(self, row):
        '''
        Function to remove an item from the queue
        '''
        pass

    def removeQueueArtifact(self, artifact):
        self.remove_artifact_trigger.emit(artifact)        

    def removeQueueArtifactMonitor(self, artifact):
        '''
        Function to remove an artifact from the queue
        '''

        # print "removing from queue:",isinstance(threading.current_thread(), threading._MainThread)
        # print "removing from queue:",threading.current_thread().__class__.__name__


        with self.update_queue_lock:

            #find the artifact row
            found = False
            for row in range(self.queue_table.rowCount()):

                if self.queue_table.item(row, 5).text() == artifact.unique_id: #remove the artifact from the queue
                    found = True
                    self.printMessage('Deleted artifact '+artifact.unique_id+ ' '+self.queue_table.item(row, 5).text(), self.green_message) #checked
                    self.queue_table.removeRow(self.queue_table.item(row,0).row())
                    break

                

            # self.queue_table.setSortingEnabled(True)

            
        #remove the artifact from the engine lists
        if artifact in self.gui_engine.queued_artifacts:
            self.gui_engine.queued_artifacts.remove(artifact)
        if artifact in self.gui_engine.all_artifacts:
            self.gui_engine.all_artifacts.remove(artifact)


    def manuallyAddArtifact(self):
        '''
        For BSM to manually add an artifact
        '''

        # original_timestamp = msg.original_timestamp, category = msg.category, \
        #                     position = [msg.orig_pos_x, msg.orig_pos_y, msg.orig_pos_z],
        #                     source_robot_id = msg.source_robot_id, artifact_report_id = msg.artifact_report_id, \
        #                     imgs = msg.imgs, img_stamps = msg.img_stamps

        # self.gui_engine.addArtifactManually()
        msg = Artifact()
        msg.original_timestamp = -1
        msg.category = self.artifact_categories[0]
        msg.orig_pose.position.x = -0.1
        msg.orig_pose.position.y = -0.1
        msg.orig_pose.position.z = -0.1
        msg.source_robot_id = -1
        msg.artifact_report_id = -1
        # msg.imgs = []
        # msg.img_stamps = []


        self.add_new_artifact_pub.publish(msg)


    def duplicateArtifact(self):
        '''
        Function to duplicate the row that has been clicked on
        '''

        if (self.displayed_artifact == None):
            self.printMessage('No row has been selected, please press this button after selecting a row in the queue.', self.normal_message) #checked

        else: #we actually have something selected
            self.gui_engine.duplicateArtifact(self.displayed_artifact)


        




    def initArtifactQueue(self, pos):
        '''
        Panel to display the queue of artifacts which have not yet been submitted
        but may be in the future
        '''
        #define the overall widget
        self.queue_widget = QWidget()
        self.queue_layout = qt.QGridLayout()

        queue_label = qt.QLabel()
        queue_label.setText('ARTIFACT QUEUE')
        queue_label.setAlignment(Qt.AlignCenter)
        self.queue_layout.addWidget(queue_label, 0, 0, 1, 3)

        #add the sort on/off button
        self.queue_table_sort_button = qt.QPushButton("Sort by time")
        self.queue_table_sort_button.setCheckable(True) # a button pressed will stay pressed, until unclicked
        self.queue_table_sort_button.toggle() #start with it sorting the table
        self.queue_layout.addWidget(self.queue_table_sort_button, 1, 0)

        #add the insert new artifact button
        self.queue_insert_artifact_button = qt.QPushButton("Add artifact")
        self.queue_insert_artifact_button.clicked.connect(self.manuallyAddArtifact)
        self.queue_layout.addWidget(self.queue_insert_artifact_button, 1, 1)

        #add the duplicate new artifact button
        self.queue_duplicate_artifact_button = qt.QPushButton("Duplicate artifact")
        self.queue_duplicate_artifact_button.clicked.connect(self.duplicateArtifact)
        self.queue_layout.addWidget(self.queue_duplicate_artifact_button, 1, 2)

        #add support for deleting artifacts
        self.queue_delete_artifact_button = qt.QPushButton("Delete artifact")
        self.queue_delete_artifact_button.clicked.connect(self.deleteArtifact)
        self.queue_layout.addWidget(self.queue_delete_artifact_button, 2, 0)

        self.queue_delete_confirm_button = qt.QPushButton("Confirm")
        self.queue_delete_confirm_button.setCheckable(True) # a button pressed will stay pressed, until unclicked
        self.queue_delete_confirm_button.clicked.connect(self.confirmDeleteArtifact)
        self.queue_delete_confirm_button.setStyleSheet("background-color:rgb(126, 126, 126)")
        self.queue_delete_confirm_button.setEnabled(False)
        self.queue_layout.addWidget(self.queue_delete_confirm_button, 2, 2)

        self.queue_delete_cancel_button = qt.QPushButton("Cancel")
        self.queue_delete_cancel_button.setCheckable(True) # a button pressed will stay pressed, until unclicked
        self.queue_delete_cancel_button.clicked.connect(self.cancelDeleteArtifact)
        self.queue_delete_cancel_button.setStyleSheet("background-color:rgb(126, 126, 126)")
        self.queue_delete_cancel_button.setEnabled(False)
        self.queue_layout.addWidget(self.queue_delete_cancel_button, 2, 1)

	#add support for requesting that all artifact info gets sent back
        self.resend_art_data_button = qt.QPushButton("Re-send all artifact info")
        self.resend_art_data_button.clicked.connect(self.resendArtifactInfo)
        self.queue_layout.addWidget(self.resend_art_data_button, 3, 0, 1, 3)


        #make a table
        self.queue_table = qt.QTableWidget()

        #resize the cells to fill the widget 
        self.queue_table.horizontalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)
        # self.queue_table.verticalHeader().setSectionResizeMode(qt.QHeaderView.ResizeToContents)
        # self.queue_table.resizeRowsToContents() #to enable word wrapping on the categories

        self.queue_table.setColumnCount(6) # set column count        
        self.queue_table.setHorizontalHeaderLabels(['Robot\nNum', 'Priority', 'Detect\nTime', '   Category   ', 'Unread', 'Unique ID']) #make the column headers

        #hide the unique_id
        self.queue_table.setColumnHidden(5,True)

        #resize the column heading depending on the content
        # header = self.queue_table.horizontalHeader()
        # header.setSectionResizeMode(3, qt.QHeaderView.Stretch)
        # header.setSectionResizeMode(4, qt.QHeaderView.ResizeToContents)


        #make sortable
        self.queue_table.setSortingEnabled(True)


        #add click listener
        self.queue_table.cellClicked.connect(self.queueClick)


        #add the table to the layout
        self.queue_layout.addWidget(self.queue_table, 4, 0, 1, 3)

        #add to the overall gui
        self.queue_widget.setLayout(self.queue_layout)
        self.global_widget.addWidget(self.queue_widget, pos[0], pos[1], pos[2], pos[3]) #last 2 parameters are rowspan and columnspan

    def resendArtifactInfo(self):
	'''
	We want all of the artifact data to be re-sent. TODO for Bob 
	is to delete the data the gui originally had
	'''
	
	self.ros_gui_bridge.resendArtifactMsg()

    def deleteArtifact(self):
        '''
        Start the process of deleting an artifact from the queue
        '''

        #confirm we've actually selected an artifact
        if (self.displayed_artifact == None):
            self.printMessage('No artifact selected, did not delete anything', self.normal_message) #checked

        #make the confirm/delete buttons pressable and the correct color
        else:
            self.queue_delete_cancel_button.setStyleSheet("background-color:rgb(220, 0, 0)")
            self.queue_delete_cancel_button.setEnabled(True)

            self.queue_delete_confirm_button.setStyleSheet("background-color:rgb(0, 220, 0)")
            self.queue_delete_confirm_button.setEnabled(True) 




    def confirmDeleteArtifact(self):
        '''
        Delete an artifact from the queue
        '''

        self.removeQueueArtifact(self.displayed_artifact)

        self.queue_delete_cancel_button.setStyleSheet("background-color:rgb(126, 126, 126)")
        self.queue_delete_cancel_button.setEnabled(False)

        self.queue_delete_confirm_button.setStyleSheet("background-color:rgb(126, 126, 126)")
        self.queue_delete_confirm_button.setEnabled(False)

    def cancelDeleteArtifact(self):
        '''
        Cancel a request to delete an artifact
        '''

        self.queue_delete_cancel_button.setStyleSheet("background-color:rgb(126, 126, 126)")
        self.queue_delete_cancel_button.setEnabled(False)

        self.queue_delete_confirm_button.setStyleSheet("background-color:rgb(126, 126, 126)")
        self.queue_delete_confirm_button.setEnabled(False)

    def initBigRed(self, pos):
        '''
        Initialize the big red button to e-stop all of the robots
        '''
        #define the overall widget
        self.bigred_widget = QWidget()
        self.bigred_layout = qt.QVBoxLayout()

        button = qt.QPushButton("SOFT ESTOP ALL\n  ROBOTS")
        button.setStyleSheet("background-color: red")
        button.clicked.connect(self.processBigRed)
        self.bigred_layout.addWidget(button)            

        #add to the overall gui
        self.bigred_widget.setLayout(self.bigred_layout)
        self.global_widget.addWidget(self.bigred_widget, pos[0], pos[1])

    def processBigRed(self):
        '''
        If the big red button is pressed, soft estop every robot
        '''

        for i, robot_name in enumerate(self.ros_gui_bridge.robot_names):
            self.control_buttons[i][2].setChecked(True) #press the button in the gui
            if (robot_name.find('erial') != -1):
                self.processRobotCommandPress(self.ros_gui_bridge.remap_from_aerial_commands[self.control_buttons[i][2].text()], robot_name, self.control_buttons[i][2])#soft estop command
            else:
                self.processRobotCommandPress(self.control_buttons[i][2].text(), robot_name, self.control_buttons[i][2])#soft estop command



    def initInfoPanel(self, pos):
        '''
        Table to display information (score, artifacts proposed, etc.) about our run
        '''

        #define the overall widget
        self.info_widget = QWidget()
        self.info_layout = qt.QVBoxLayout()

        boldFont = gui.QFont("", 16, gui.QFont.Bold) 


        self.info_label = qt.QLabel()
        self.info_label.setText('Time: -- \t Score: -- \t Proposals Left: --')
        self.info_label.setAlignment(Qt.AlignCenter)
        self.info_label.setFont(boldFont)
        self.info_label.setStyleSheet('border:3px solid rgb(0, 0, 0);')

        self.info_layout.addWidget(self.info_label)

        

        #add to the overall gui
        self.info_widget.setLayout(self.info_layout)
        self.global_widget.addWidget(self.info_widget, pos[0], pos[1], pos[2], pos[3])

    def initArtHistoryPanel(self, pos):
        '''
        Table to display info regarding submissions of artifacts
        '''

        #define the overall widget
        self.arthist_widget = QWidget()
        self.arthist_layout = qt.QGridLayout()

        arthist_label = qt.QLabel()
        arthist_label.setText('ARTIFACT SUBMISSION INFO')
        arthist_label.setAlignment(Qt.AlignCenter)
        self.arthist_layout.addWidget(arthist_label, 0, 0)

        #add the sort on/off button
        self.arthist_table_sort_button = qt.QPushButton("Sort by time")
        self.arthist_table_sort_button.setCheckable(True) # a button pressed will stay pressed, until unclicked
        self.arthist_table_sort_button.toggle() #start with it sorting the table

        self.arthist_layout.addWidget(self.arthist_table_sort_button, 1, 0)

         #make a table
        self.arthist_table = qt.QTableWidget()


        #resize the cells to fill the widget 
        self.arthist_table.horizontalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)
        # self.arthist_table.verticalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)

        self.arthist_table.setColumnCount(4) # set column count        
        self.arthist_table.setHorizontalHeaderLabels(['Category', 'Time', 'x/y/z', 'Response']) #make the column headers

        #resize the column heading depending on the content
        # header = self.arthist_table.horizontalHeader()
        # header.setSectionResizeMode(0, qt.QHeaderView.Stretch)
        # header.setSectionResizeMode(1, qt.QHeaderView.ResizeToContents)
        # header.setSectionResizeMode(2, qt.QHeaderView.Stretch)
        # header.setSectionResizeMode(3, qt.QHeaderView.ResizeToContents)

        #make sortable
        self.arthist_table.setSortingEnabled(True)

        #init the artifact submission response list
        self.art_responses = ['Response1.1']

        tooltip_font = gui.QFont()
        tooltip_font.setPointSize(12)


        #add the table to the layout
        self.arthist_layout.addWidget(self.arthist_table)

        #add to the overall gui
        self.arthist_widget.setLayout(self.arthist_layout)
        self.global_widget.addWidget(self.arthist_widget, pos[0], pos[1], pos[2], pos[3]) 

    def displaySeconds(self, seconds):
        '''
        Function to convert seconds float into a min:sec string
        '''
        return str((int(float(seconds))/60))+':'+str(int(float(seconds)-(int(float(seconds))/60)*60))
      

    def initMessagePanel(self, pos):
        '''
        Initialize the pnale which displays ros loginfo messages
        '''

        #define the overall widget
        self.message_box_widget = QWidget()
        self.message_box_layout = qt.QGridLayout()

        message_label = qt.QLabel()
        message_label.setText('MESSAGE PANEL')
        message_label.setAlignment(Qt.AlignCenter)
        self.message_box_layout.addWidget(message_label, 0, 0)


        self.message_textbox = qt.QListWidget()
        self.message_textbox.setWordWrap(True)

        self.message_box_layout.addWidget(self.message_textbox, 1, 0)


        #add to the overall gui
        self.message_box_widget.setLayout(self.message_box_layout)
        self.global_widget.addWidget(self.message_box_widget, pos[0], pos[1], pos[2], pos[3]) 

    def addMessage(self, msg):
        self.add_message_trigger.emit(msg)

    def addMessageMonitor(self, msg):
        '''
        Add a message to the message box from a rostopic
        '''
        # print "add message monitor:",isinstance(threading.current_thread(), threading._MainThread)
        # print "add message monitor:",threading.current_thread().__class__.__name__

        with self.update_message_box_lock:
            self.message_textbox.setSortingEnabled(False)
            
            if (self.darpa_gui_bridge.darpa_status_update['run_clock'] != None):
                self.message_textbox.addItem('['+str(self.darpa_gui_bridge.displaySeconds(self.darpa_gui_bridge.darpa_status_update['run_clock']))+'] '+msg.data)
            else:
                self.message_textbox.addItem('[--] '+msg.data)

            self.message_textbox.setSortingEnabled(True)

            self.message_textbox.sortItems(core.Qt.DescendingOrder)

            self.message_textbox.viewport().update()

    def printMessage(self, msg, rgb):
        self.print_message_trigger.emit(msg, rgb)


    def printMessageMonitor(self, msg, rgb=None):
        '''
        Add message to the messag box that is simply a string from
        this application (not ROS)
        '''

        # print "print message:",isinstance(threading.current_thread(), threading._MainThread)
        # print "print message:",threading.current_thread().__class__.__name__
        
        with self.update_message_box_lock:
            self.message_textbox.setSortingEnabled(False)


            if (self.darpa_gui_bridge != None and self.darpa_gui_bridge.darpa_status_update['run_clock'] != None):
                item = qt.QListWidgetItem('['+str(self.darpa_gui_bridge.displaySeconds(self.darpa_gui_bridge.darpa_status_update['run_clock']))+'] '+msg)
            else:
                item = qt.QListWidgetItem('[--] '+msg)

            if (rgb != None):
                item.setBackground(gui.QColor(rgb[0], rgb[1], rgb[2]))

            self.message_textbox.addItem(item)



            self.message_textbox.setSortingEnabled(True)

            self.message_textbox.sortItems(core.Qt.DescendingOrder)

            self.message_textbox.viewport().update()




    def buildGui(self):
        '''
        Function to layout the gui (place widgets, etc.)
        '''

        # self.init_buttons(self.config_filename)

        #define the position of everything in terms of row, column
        info_pos    = [0,1,1,1]
        bigred_pos  = [0,2]
        status_pos  = [3,2,2,1]
        queue_pos   = [1,0,4,1] #last 2 parameters are rowspan and columnspan
        control_pos = [5,2,2,1]
        artvis_pos  = [1,1,4,1]
        arthist_pos = [5,0,2,1]     
        artmanip_pos = [5,1,1,1] 
        message_pos = [1,2,2,1]   
        

        #initialize the panels
        self.initControlPanel(control_pos) #the panel to send robots commands (e-stop, etc.)
        self.initArtifactVisualizer(artvis_pos) #panel to display and react to artifacts as they come in
        self.initStatusPanel(status_pos) #panel to display health info of each robot
        self.initArtifactQueue(queue_pos) #panel to display the artifact queue
        self.initBigRed(bigred_pos) #the big red button to e-stop all of the robots
        self.initInfoPanel(info_pos) #table to display information (score, artifacts proposed, etc.) about our run
        self.initArtHistoryPanel(arthist_pos) #table to display artifact proposals
        self.initArtifactManipulator(artmanip_pos) #panel to complete actions w.r.t. artifacts
        self.initMessagePanel(message_pos) #panel to display ros loginfo messages

        #initialize the subscribers for updating different parts of the GUI
        # self.info_subscriber = rospy.Subscriber('/darpa_status_updates', String, self.updateInfoPanel)

        self.status_panel_update_sub = rospy.Subscriber('/gui/status_panel_update', StatusPanelUpdate, self.status_panel_update_callback)



        #load the darpa transform
        self.loadDarpaTransform()

    def updateInfoPanel(self, msg):
        self.update_info_panel_trigger.emit(msg)

    def updateInfoPanelMonitor(self, msg):
        '''
        Subscriber that constantly updates info panel
        '''

        # print "update info panel:",isinstance(threading.current_thread(), threading._MainThread)
        # print "update info panel:",threading.current_thread().__class__.__name__

        
        #update the info panel
        self.info_label.setText(msg)

        #save the state of the gui
        if (self.save_count == 30): #to make sure we save every x seconds
            self.gui_engine.savePeriodically(self)
            self.save_count = 0
        else:
            self.save_count += 1

    def fillInGuiFromCsv(self, filename):
        '''
        From a selected filename, fill in the various gui components
        '''

        # print "fill in gui:",isinstance(threading.current_thread(), threading._MainThread)
        # print "fill in gui:",threading.current_thread().__class__.__name__

        #all we really need is the last line
        csv_data = None
        with open(filename, 'r') as f:
            for row in reversed(list(csv.reader(f))):
                csv_data = row
                break

        if (csv_data == None):
            self.printMessage("There was nothing in the csv file!!", self.red_message)

        else:
            #parse the artifacts
            artifact_data = csv_data[1]
            split_art_data = artifact_data.split('//')

            for art_str in split_art_data:

                split_artifact = art_str.split('|')

                if (len(split_artifact)>12): #if we actually have something here

                    #build the artifact object
                    artifact = Artifact()
                    artifact.category = split_artifact[0]
                    artifact.pos = [float(i) for i in split_artifact[1].split(',')]
                    artifact.orig_pos = [float(i) for i in split_artifact[2].split(',')]
                    artifact.source_robot = int(split_artifact[3])
                    artifact.artifact_report_id = int(split_artifact[4])
                    artifact.time_from_robot = float(split_artifact[5]) #time the detection has come in from the robot. TODO: change to be something different?
                    artifact.time_to_darpa = float(split_artifact[6]) #time submitted to darpa
                    artifact.unread = (split_artifact[7] == 'True')
                    artifact.priority = split_artifact[8]

                    if (split_artifact[9] != ''): #if we actually have a darpa response
                        artifact.darpa_response = split_artifact[9][:split_artifact[9].find('HTTP')]+'\n'+\
                                                  split_artifact[9][split_artifact[9].find('HTTP') : split_artifact[9].find('Submission')] +'\n'+\
                                                  split_artifact[9][split_artifact[9].find('Submission') : ]
                    else:
                        artifact.darpa_response = None

                    artifact.unique_id = split_artifact[10]
                    artifact.original_timestamp = split_artifact[11]

                    
                    if (artifact.darpa_response != None): #this artifact has been submitted to darpa

                        with self.artifact_proposal_lock:

                            self.arthist_table.setSortingEnabled(False) 

                            #add to the engine
                            self.gui_engine.all_artifacts.append(artifact)
                            self.gui_engine.submitted_artifacts.append(artifact)

                            #add to the submission panel
                            submission_time = self.darpa_gui_bridge.displaySeconds(artifact.time_to_darpa)

                            submission_correct = artifact.darpa_response[artifact.darpa_response.find('?')+1:].replace(' ','')
                            
                            if (submission_correct=='False'):
                                submission_color = gui.QColor(220,0,0)
                            else:
                                submission_color = gui.QColor(0,220,0)

                            response_item = qt.QTableWidgetItem('Info')
                            response_item.setToolTip(artifact.darpa_response)
                            response_item.setBackground(submission_color)

                            self.arthist_table.insertRow(self.arthist_table.rowCount())
                            row = self.arthist_table.rowCount() - 1


                            row_data = [artifact.category, submission_time, str(int(artifact.pos[0]))+'/'+str(int(artifact.pos[1]))+'/'+str(int(artifact.pos[2]))]

                            for col, val in enumerate(row_data):

                                if (col==1):
                                    colon = submission_time.find(':')
                                    val = float(submission_time[:colon])*60 + float(submission_time[colon+1:])
                                    item = NumericItem(str(submission_time))
                                    item.setData(core.Qt.UserRole, val)
                            
                                else: #if we're not dealing with a display time
                                    item = NumericItem(str(val))
                                    item.setData(core.Qt.UserRole, val)                                   
                                
                                item.setFlags( core.Qt.ItemIsSelectable |  core.Qt.ItemIsEnabled ) #make it non-editable
                                item.setTextAlignment(Qt.AlignHCenter) 

                                self.arthist_table.setItem(row, col, item)

                            #add the response item, we don't want this to be a NumericItem
                            self.arthist_table.setItem(row, 3, response_item)

                            self.arthist_table.setSortingEnabled(True) 

                            if(self.arthist_table_sort_button.isChecked()): #if the sort button is pressed, sort the incoming artifacts
                                self.arthist_table.sortItems(1, core.Qt.DescendingOrder)

                                self.arthist_table.viewport().update()



                    else: #this artifact is still in the queue 

                        #if there does not exist the same (now updated) artifact already in the queue
                        found_match = False

                        for arti in self.gui_engine.queued_artifacts:
                            if (artifact.unique_id == arti.unique_id):
                                found_match = True

                        if (not found_match):

                            #add the artifact to the list of queued objects and to the all_artifacts list
                            self.gui_engine.queued_artifacts.append(artifact)
                            self.gui_engine.all_artifacts.append(artifact)

                            #call a function to graphically add it to the queue
                            self.sendToQueue(artifact)



            #parse the vehicle states
            vehicle_data = csv_data[2] 

            vehicle_data_split = vehicle_data.split('|')

            for i, command in enumerate(vehicle_data_split):

                if (i> 0 and len(command)>1): #we actually have something here
                    # print command

                    #go find the button this corresponds to and "press" it
                    found_button = False

                    for button in self.control_buttons[i]:
                        if(button.text() == command):
                            found_button = True
                            button.click()

                    if(found_button != True):
                        self.printMessage("Could not find the appropriate command button to be pressed. Gui may not be filled in properly", self.red_message) #checked


            #parse the info
            info_data = csv_data[3] 


            #save periodically
            self.gui_engine.savePeriodically(self)

            self.printMessage("Data loaded from csv", self.green_message) #checked


    

    def status_panel_update_callback(self, msg):
        pass
        # self.status_panel_trigger.emit(msg)#self.emit(SIGNAL("changeUI(PyQt_PyObject)"), msg)

    def status_panel_update_monitor(self, msg):

        # print "status panel:",isinstance(threading.current_thread(), threading._MainThread)
        # print "status panel:",threading.current_thread().__class__.__name__

        # print "start status", msg.value, type(msg.value)
        msg_str = [ord(c) for c in msg.value]
        # print msg_str
        # print msg.color.r, msg.color.g, msg.color.b
        column_index = msg.robot_id
        # print "1"
        if column_index < 0 or column_index >= self.status_table.columnCount():
            rospy.logerr('robot_id %d is out of bounds: ' % column_index)
            return
        # print "2"
        if msg.key not in self.statuses:
            rospy.logerr('key %s is not the name of a row in the status panel' % msg.key)
            return
        # print "3"
        row_index = self.statuses.index(msg.key)
# 
        # print "4"

        item = self.status_table.item(row_index, column_index)
        
        if item == None:
            self.status_table.setItem(row_index, column_index, qt.QTableWidgetItem(''))
        item = self.status_table.item(row_index, column_index)
        item.setBackground(gui.QColor(msg.color.r, msg.color.g, msg.color.b))
        # if (row_index!=0):
        #     item.setText(msg.value)
        # else:
        #     item.setText('hello')
        item.setText(msg.value)
        # print "5"
        self.status_table.viewport().update()

        # print "end status"


    def select_config_file(self):
        starting_path = os.path.join(rospkg.RosPack().get_path('basestation_gui_python'), 'config')
        filename = qt.QFileDialog.getOpenFileName(self.widget, 'Open Config File', starting_path, "Config Files (*.yaml)")[0]
        if filename != '':
            self.initiateSettings(filename)

    def loadFromCsv(self):
        '''
        Function for loading the gui state from a csv file (used if
        gui crashes or something like that)
        '''
        starting_path = os.path.join(rospkg.RosPack().get_path('basestation_gui_python'), 'custom_logs')
        filename = qt.QFileDialog.getOpenFileName(self.widget, 'Load From CSV', starting_path, "CSV (*.csv)")[0]
        if (filename == ''):
            self.printMessage('Unable to load csv, did not find one by that name.', self.red_message)
        else:
            self.fillInGuiFromCsv(filename)
            

    def state_callback(self, msg):
        try:
            state_name = msg.data
            self.state_label.setText('STATE: ' + state_name)
            for transition in self.ros_gui_bridge.get_transitions():
                if self.ros_gui_bridge.states[state_name].has_transition(transition):
                    self.buttons[transition].setEnabled(True)
                else:
                    self.buttons[transition].setEnabled(False)
        except:
            pass
    
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        if(self.connect_to_command_post):
            self.darpa_gui_bridge.shutdownHttpServer()
        

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('config_filename', self.config_filename)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        '''
        Function which lays out all of the widgets from a pre-specified .yaml 
        config file. 
        '''
        self.initiateSettings(instance_settings.value('config_filename'))
        
    def defineRobotTransformButtons(self):
        '''
        After we define the rosgui bridge, generate bttons for their transforms
        '''

        self.first_save_pose_button = qt.QPushButton('Save robot/total \npose: '+self.ros_gui_bridge.robot_names[0])
        self.first_save_pose_button.clicked.connect(partial(self.saveRobotPoseGui, 0))
        self.top_layout.addWidget(self.first_save_pose_button, 1,0,)

        self.second_save_pose_button = qt.QPushButton('Save robot/total \npose: '+self.ros_gui_bridge.robot_names[1])
        self.second_save_pose_button.clicked.connect(partial(self.saveRobotPoseGui, 1))
        self.top_layout.addWidget(self.second_save_pose_button, 1,1)

    def initiateSettings(self, config_filename):
        '''
        Generates the gui using either start fresh or from using previous settings
        '''


        self.config_filename = config_filename
        self.ros_gui_bridge = RosGuiBridge(self.config_filename, self)

        self.defineRobotTransformButtons()
        
        config = yaml.load(open(self.config_filename, 'r').read())
        darpa_params = config['darpa_params']
        self.associateCatsWithLabels(darpa_params['artifact_labels'])

        #read info on darpa-related commands (communication protocol, etc.)
        darpa_params = config['darpa_params']

        #artifact categories used for the gui
        self.artifact_categories = []
        for category in darpa_params['artifact_categories']:
            self.artifact_categories.append(category)

        self.darpa_gui_bridge = DarpaGuiBridge(self.config_filename, self)

        self.buildGui()        
        
        self.gui_engine = GuiEngine(self)

        self.ros_gui_bridge.initSubscribers(self)



    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog






