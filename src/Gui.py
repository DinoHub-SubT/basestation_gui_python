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
        self.top_layout.addWidget(self.load_from_csv, 0, 1)

        self.save_pose_button = qt.QPushButton('Save robot pose')
        self.save_pose_button.clicked.connect(self.saveRobotPoseGui)
        self.top_layout.addWidget(self.save_pose_button, 1,0)


        self.global_widget.addWidget(self.top_widget)
        
        
        self.widget.setLayout(self.global_widget)
        context.add_widget(self.widget)

        self.artifact_proposal_lock = threading.Lock()
        self.update_queue_lock = threading.Lock()

        self.displayed_artifact = None #which artifact is currently being displayed

        self.dont_change_art_priority = False #if we click in the artifact  queue, just update the gui and nothing else
        self.dont_change_art_category = False

        self.save_count = 0 #way to ensure we don't save the gui too often



    def saveRobotPoseGui(self):
        '''
        When a button is pressed, its pose gets saved to a textfile
        '''
        rospack = rospkg.RosPack()
        pose_filename = rospack.get_path('basestation_gui_python')+'/ji_pose.csv'

        robot_pos = self.ros_gui_bridge.getRobotPose()

        if (robot_pos != None):
            with open(pose_filename, 'a') as writeFile:
                writer = csv.writer(writeFile)
                writer.writerow([robot_pos[0], robot_pos[1], robot_pos[2]]) 

        print "Saved robot pose."






    def initControlPanel(self, pos):
        '''
        Initial the panel containing e-stop command, etc for each robot
        '''

        #define the overall widget
        self.control_widget = QWidget()
        self.control_layout = qt.QGridLayout()
        self.control_buttons = []

        control_label = qt.QLabel()
        control_label.setText('CONTROL PANEL')
        control_label.setAlignment(Qt.AlignCenter)
        self.control_layout.addWidget(control_label)

        #define the number of commands in a single column
        num_in_col = 4

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

                button = qt.QPushButton(command)
                button.setCheckable(True) # a button pressed will stay pressed, until unclicked
                button.setStyleSheet("QPushButton:checked { background-color: red }") #a button stays red when its in a clicked state
                robot_button_list.append(button)
                
                
                #upon press, do something in ROS
                button.clicked.connect(partial(self.processRobotCommandPress, command, robot_name, button))

                robot_layout.addWidget(button, row, col)

                row+=1

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
        if (button.text() in self.ros_gui_bridge.estop_commands):

            #find the set of control buttons for this robot
            robot_ind = -1
            for i, name in enumerate(self.ros_gui_bridge.robot_names):
                if (robot_name == name):
                    robot_ind = i


            if (robot_ind!=-1):
                control_buttons = self.control_buttons[robot_ind]

                # print control_buttons

                for cmd_button in control_buttons:
                    if (cmd_button != button) and  (cmd_button.text() in self.ros_gui_bridge.estop_commands):
                        
                        cmd_button.setChecked(False)


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
        art_label.setText('ARTIFACT PANEL')
        art_label.setAlignment(Qt.AlignCenter)
        self.artvis_layout.addWidget(art_label, 0, 0, 1, 4)

        #add in a blank label to represent an object
        self.art_image = qt.QLabel()

        # img = cv2.imread('/home/bob/basestation_ws/src/basestation_gui_python/fake_artifact_imgs/test_img.jpg')
        
        rospack = rospkg.RosPack()
        img_filename = rospack.get_path('basestation_gui_python')+'/src/black_img.png'

        img = cv2.imread(img_filename)
        img = cv2.resize(img,(600, 450))

        img_height, img_width = img.shape[:2]
        img = gui.QImage(img, img_width, img_height, gui.QImage.Format_RGB888)
        img = gui.QPixmap.fromImage(img)
        self.art_image.setPixmap(img)

        self.artvis_layout.addWidget(self.art_image, 1, 0, 1, 4) #last 2 parameters are rowspan and columnspan

        

        #add to the overall gui
        self.artvis_widget.setLayout(self.artvis_layout)
        self.global_widget.addWidget(self.artvis_widget, pos[0], pos[1], pos[2], pos[3])

    def displayArtifactImg(self, artifact, index = 0):
        '''
        Display an artifact's index-th image
        '''

        if (index+1) > len(artifact.imgs):
            rospack = rospkg.RosPack()
            img_filename = rospack.get_path('basestation_gui_python')+'/src/black_img.png'

            img = cv2.imread(img_filename)
       
        else:
            img = artifact.imgs[index]
        
        img = cv2.resize(img,(600, 450))

        img_height, img_width = img.shape[:2]
        img = gui.QImage(img, img_width, img_height, gui.QImage.Format_RGB888)
        img = gui.QPixmap.fromImage(img)
        self.art_image.setPixmap(img)

    def processArtRefinementPress(self):
        '''
        Process pressing of the button, which visualizes the 
        interactive marker for artifact refinement
        '''
        if (self.displayed_artifact != None):
            self.ros_gui_bridge.publishRefinementMarkerPos(self.displayed_artifact, self.art_refinement_button)
        else:
            print "Request not processed. No artifact currently being displayed"
        
    def updateRefinmentPos(self, msg):
        '''
        When the interactive marker moves, this 
        is called to update the appropriate textboxes and 
        update the artifact position
        '''

        #change the textboxes
        self.art_pos_textbox_x.setText(str(msg.pose.position.x)[:7])
        self.art_pos_textbox_y.setText(str(msg.pose.position.y)[:7])
        self.art_pos_textbox_z.setText(str(msg.pose.position.z)[:7])

        #change the artifact 
        if(self.displayed_artifact != None):
            self.displayed_artifact.pos[0] = msg.pose.position.x
            self.displayed_artifact.pos[1] = msg.pose.position.y
            self.displayed_artifact.pos[2] = msg.pose.position.z



    def initArtifactManipulator(self, pos):
        '''
        Buttons and textboxes to refine the position and complete actions with the  artifact
        '''

         #define the overall widget
        self.artmanip_widget = QWidget()
        self.artmanip_layout = qt.QGridLayout()

        #add button for displaying the interactive marker
        self.art_refinement_button = qt.QPushButton("Show Refinement Marker")
        self.art_refinement_button.setCheckable(True)
        self.art_refinement_button.clicked.connect(self.processArtRefinementPress)
        self.artmanip_layout.addWidget(self.art_refinement_button, 0, 0 , 1, 4)

        #add in information about 3d position     
        dimensions = ['X', 'Y', 'Z'] 

        boldFont = gui.QFont()
        boldFont.setBold(True)

        for i,dim in enumerate(dimensions):
            dim_label = qt.QLabel()
            dim_label.setText(dim)
            dim_label.setAlignment(Qt.AlignCenter)
            dim_label.setFont(boldFont)
            self.artmanip_layout.addWidget(dim_label, 1, i+1)

        #information about the detected position
        robot_pos = ['N/A', 'N/A', 'N/A'] #fake data

        self.orig_pos_label = qt.QLabel()
        self.orig_pos_label.setText('Original Position')
        self.artmanip_layout.addWidget(self.orig_pos_label, 2, 0)

        self.orig_pos_label_x = qt.QLabel()
        self.orig_pos_label_x.setText(str(robot_pos[0]))
        self.orig_pos_label_x.setAlignment(Qt.AlignCenter)
        self.artmanip_layout.addWidget(self.orig_pos_label_x, 2, 1)

        self.orig_pos_label_y = qt.QLabel()
        self.orig_pos_label_y.setText(str(robot_pos[1]))
        self.orig_pos_label_y.setAlignment(Qt.AlignCenter)
        self.artmanip_layout.addWidget(self.orig_pos_label_y, 2, 2)

        self.orig_pos_label_z = qt.QLabel()
        self.orig_pos_label_z.setText(str(robot_pos[2]))
        self.orig_pos_label_z.setAlignment(Qt.AlignCenter)
        self.artmanip_layout.addWidget(self.orig_pos_label_z, 2, 3)

        #editable information about the position, to send to darpa
        refined_pos_label = qt.QLabel()
        refined_pos_label.setText('Refined Position')
        self.artmanip_layout.addWidget(refined_pos_label, 3, 0)

        self.art_pos_textbox_x, self.art_pos_textbox_y, self.art_pos_textbox_z = qt.QLineEdit(), qt.QLineEdit(), qt.QLineEdit()
       
        #fill in some fake data
        self.art_pos_textbox_x.setText(str(robot_pos[0]))
        self.artmanip_layout.addWidget(self.art_pos_textbox_x, 3, 1)

        self.art_pos_textbox_y.setText(str(robot_pos[1]))
        self.artmanip_layout.addWidget(self.art_pos_textbox_y, 3, 2)

        self.art_pos_textbox_z.setText(str(robot_pos[2]))
        self.artmanip_layout.addWidget(self.art_pos_textbox_z, 3, 3)



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
        self.artmanip_layout.addWidget(art_action_label2, 4, 0, 1, 3)

        art_action_label3 = qt.QLabel()
        art_action_label3.setText('\n\nPriority')
        art_action_label3.setFont(boldFont)
        art_action_label3.setAlignment(Qt.AlignCenter)
        self.artmanip_layout.addWidget(art_action_label3, 4, 3, 1, 2)

        button = qt.QPushButton("To DARPA")
        button.clicked.connect(partial(self.proposeArtifact))
        self.artmanip_layout.addWidget(button, 6, 0, 1, 4)

        # button = qt.QPushButton("ARCHIVE")
        # # button.setSizePolicy(QSizePolicy.Expanding, 0)
        # self.artmanip_layout.addWidget(button, 6,0,1,4)

        #make the combobox for setting the artifact category
        self.darpa_cat_box = qt.QComboBox()
        
        for category in self.ros_gui_bridge.artifact_categories:
            self.darpa_cat_box.addItem(category)
        
        self.darpa_cat_box.currentTextChanged.connect(self.updateArtifactCat)

        self.artmanip_layout.addWidget(self.darpa_cat_box, 5, 0, 1, 3)

        #make the combobox for setting the artifact priority
        self.artifact_priority_box = qt.QComboBox() 

        self.artifact_priority_box.addItem('High')
        self.artifact_priority_box.addItem('Med')
        self.artifact_priority_box.addItem('Low')

        
        self.artifact_priority_box.currentTextChanged.connect(self.updateArtifactPriority)

        self.artmanip_layout.addWidget(self.artifact_priority_box, 5, 3, 1, 1)



        # button = qt.QPushButton("    Discard    ")
        # # button.setSizePolicy(QSizePolicy.Expanding, 0)
        # self.artmanip_layout.addWidget(button, 6, 0, 1, 2)

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
            print "Nothing proposed. Please select an artifact"

        elif (self.displayed_artifact==None):
            print "Nothing proposed. No artifact being displayed. Please select an artifact"

        elif(self.connect_to_command_post):
            
            with self.artifact_proposal_lock: #to ensure we only draw one response at once

                self.arthist_table.setSortingEnabled(False) #to avoid corrupting the table
            
                data = [ float(self.art_pos_textbox_x.text()), float(self.art_pos_textbox_y.text()), \
                         float(self.art_pos_textbox_z.text()), self.darpa_cat_box.currentText()]

                print "\n\nSubmitted cat: --"+self.darpa_cat_box.currentText()+'--\n\n'

                
                proposal_return = self.darpa_gui_bridge.startArtifactProposal(data)

                if(len(proposal_return) > 0):
                    [submission_time_raw, artifact_type, x, y, z, report_status, score_change, http_response, http_reason] = \
                                                                                                    proposal_return

                    #add this to the submission history panel 
                    submission_time = self.darpa_gui_bridge.displaySeconds(float(submission_time_raw))

                    if(score_change==0):
                        submission_correct='False'
                        submission_color = gui.QColor(220,0,0)
                    else:
                        submission_correct='True'
                        submission_color = gui.QColor(0,220,0)

                    response_item = qt.QTableWidgetItem('Info')
                    response_item.setToolTip('DARPA response: '+str(report_status)+'\nHTTP Response: '+str(http_response)+str(http_reason)+\
                                             '\nSubmission Correct? '+submission_correct)
                    response_item.setBackground(submission_color)

                    #update the artifact's darpa_response property
                    self.displayed_artifact.darpa_response = 'DARPA response: '+str(report_status)+'\nHTTP Response: '+\
                                                             str(http_response)+str(http_reason)+\
                                                             '\nSubmission Correct? '+submission_correct

                    #update the artifacts' to_darpa time
                    self.displayed_artifact.time_to_darpa = int(float(submission_time_raw))


                    #add the stuff to the submission history table

                    self.arthist_table.insertRow(self.arthist_table.rowCount())
                    row = self.arthist_table.rowCount() - 1

                    row_data = [artifact_type, submission_time, str(int(x))+'/'+str(int(y))+'/'+str(int(z))]

                    for col, val in enumerate(row_data):
                    
                        if (str(val).find(':')==-1): #if we're not dealing with a display time
                            item = NumericItem(str(val))
                            item.setData(core.Qt.UserRole, val)
                            
                        
                        else:
                            colon = submission_time.find(':')
                            val = float(submission_time[:colon])*60 + float(submission_time[colon+1:])
                            item = NumericItem(str(submission_time))
                            item.setData(core.Qt.UserRole, val)

                        self.arthist_table.setItem(row, col, item)

                    #add the response item, we don't want this to be a NumericItem
                    self.arthist_table.setItem(row, 3, response_item)




                    #go find the artifact in the queue and remove it
                    with self.update_queue_lock:
                        self.queue_table.setSortingEnabled(False)
                        row_ind = self.findDisplayedArtifact()

                        if(row_ind!=-1):
                            self.queue_table.removeRow(self.queue_table.item(row_ind,0).row())


                            #also remove it from the gui engine artifacts list and put it into the gui engine proposed list
                            self.gui_engine.queued_artifacts.remove(self.displayed_artifact)
                            self.gui_engine.submitted_artifacts.append(self.displayed_artifact)

                        self.queue_table.setSortingEnabled(True)


                    #remove the artifact from the main visualization panel
                    self.orig_pos_label_x.setText('')
                    self.art_pos_textbox_x.setText('')

                    self.orig_pos_label_y.setText('')
                    self.art_pos_textbox_y.setText('')

                    self.orig_pos_label_z.setText('')
                    self.art_pos_textbox_z.setText('')

                    self.displayed_artifact = None


                self.arthist_table.setSortingEnabled(True) 

                if(self.arthist_table_sort_button.isChecked()): #if the sort button is pressed, sort the incoming artifacts
                        self.arthist_table.sortItems(1, core.Qt.DescendingOrder)

                
    def findDisplayedArtifact(self):
        '''
        Returns the row of the artifact table for the displayed artifact
        '''
        if (self.queue_table.isSortingEnabled()):
            print "\n\n Table did not have sorting disabled!! \n\n"
            self.queue_table.setSortingEnabled(False)

        if(self.displayed_artifact!=None): #if we have actually set the artifact
            for i in range(self.queue_table.rowCount()):
                robot_id = self.queue_table.item(i, 0).text()
                art_id = self.queue_table.item(i, 5).text()

                if (int(self.displayed_artifact.source_robot) == int(robot_id)) and \
                   (int(self.displayed_artifact.artifact_report_id) == int(art_id)):

                    return i

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


    def updateArtifactPriority(self):
        '''
        The combo box for changing the artifact priority was pressed
        self.dont_change_art_priority indicates if the change is automatic after clicking 
        on an artifact in the  queue and therefore don't do anything
        '''

        if (not self.dont_change_art_priority) and self.displayed_artifact!=None :

            self.displayed_artifact.priority = self.artifact_priority_box.currentText()

            #change the text in the queue
            self.queue_table.setSortingEnabled(False)
            
            row_ind = self.findDisplayedArtifact()

            if(row_ind!=-1):
                with self.update_queue_lock:
                    self.queue_table.setItem(row_ind, 1, qt.QTableWidgetItem(self.artifact_priority_box.currentText()))
                    self.queue_table.item(row_ind, 1).setTextAlignment(Qt.AlignHCenter) 
                    self.queue_table.item(row_ind, 1).setFlags( core.Qt.ItemIsSelectable |  core.Qt.ItemIsEnabled )

            self.queue_table.setSortingEnabled(True)

        self.dont_change_art_priority = False #reset its value


    def updateArtifactCat(self):
        '''
        The combo box for changing the artifact category was pressed
        '''

        #first check the combobox change came from a  human and not a automatic call
        if (not self.dont_change_art_category) and self.displayed_artifact!=None :             

            #change the text in the queue
            self.queue_table.setSortingEnabled(False)
            
            row_ind = self.findDisplayedArtifact()

            #ensure that there is not an artifact with the same robot_id/time/category
            found_match = False

            robot_id = int(float(self.queue_table.item(row_ind, 0).text()))
            detect_time = self.dispToSeconds(self.queue_table.item(row_ind, 2).text())
            category = self.darpa_cat_box.currentText()

            for i in range(self.queue_table.rowCount()):

                if  (int(float(self.queue_table.item(i, 0).text())) == robot_id) and \
                    (self.dispToSeconds(self.queue_table.item(i, 2).text()) == detect_time) and \
                    (self.queue_table.item(i, 3).text() == category):

                    found_match = True

            if(found_match): #we already have an artifact of this robot_id/time/category
                print "Cannot complete request. Already have an artifact of the same robot_id/time/category"

            else:
                if(row_ind!=-1):
                    with self.update_queue_lock:
                        self.queue_table.setItem(row_ind, 3, qt.QTableWidgetItem(self.darpa_cat_box.currentText()))
                        self.queue_table.item(row_ind, 3).setTextAlignment(Qt.AlignHCenter) 
                        self.queue_table.item(row_ind, 3).setFlags( core.Qt.ItemIsSelectable |  core.Qt.ItemIsEnabled )

                self.displayed_artifact.category = self.darpa_cat_box.currentText()

            self.queue_table.setSortingEnabled(True)

        self.dont_change_art_category = False #reset its value
        


    def sendToQueue(self, artifact):
        '''
        Send the artifact subscribed to, to the queue
        '''

        with self.update_queue_lock:

            #if we need to connect to the command post, wait
            if (self.connect_to_command_post and self.darpa_gui_bridge.darpa_status_update['run_clock']==None):

                #remove the artifact
                self.gui_engine.queued_artifacts.remove(artifact)
                self.gui_engine.all_artifacts.remove(artifact)
                return False

            else:

                self.queue_table.setSortingEnabled(False) #to avoid corrupting the table

                self.queue_table.insertRow(self.queue_table.rowCount())
                row = self.queue_table.rowCount() - 1

                #fill in the row data
                if (artifact.time_from_robot == -1): #this si coming directly from the robot
                    disp_time = self.darpa_gui_bridge.displaySeconds(float(self.darpa_gui_bridge.darpa_status_update['run_clock']))
                
                else:
                    disp_time = self.darpa_gui_bridge.displaySeconds(float(artifact.time_from_robot))

                # random.sample(rand_list,1)[0]

                row_data = [artifact.source_robot, artifact.priority, disp_time, \
                            artifact.category, '!', artifact.artifact_report_id]


                for col, val in enumerate(row_data):
                    
                    if (str(val).find(':')==-1): #if we're not dealing with a display time
                        item = NumericItem(str(val))
                        item.setData(core.Qt.UserRole, val)
                        
                    
                    else:
                        colon = disp_time.find(':')
                        val = float(disp_time[:colon])*60 + float(disp_time[colon+1:])
                        item = NumericItem(str(disp_time))
                        item.setData(core.Qt.UserRole, val)

                    self.queue_table.setItem(row, col, item)


                artifact.time_from_robot = int(float(self.darpa_gui_bridge.darpa_status_update['run_clock']))

                #color the unread green
                self.queue_table.item(row, 4).setBackground(gui.QColor(0,255,0))

                for i in range(self.queue_table.columnCount()): #make the cells not editable and make the text centered
                    if self.queue_table.item(row, i) != None: 
                        self.queue_table.item(row, i).setFlags( core.Qt.ItemIsSelectable |  core.Qt.ItemIsEnabled )
                        self.queue_table.item(row, i).setTextAlignment(Qt.AlignHCenter) 

                self.queue_table.setSortingEnabled(True) #to avoid corrupting the table

                if(self.queue_table_sort_button.isChecked()): #if the sort button is pressed, sort the incoming artifacts
                    self.queue_table.sortItems(2, core.Qt.DescendingOrder)


   

    def initStatusPanel(self, pos):
        '''
        Panel to display information (battery health, comms signal, etc.)
        for each robot
        '''

        #define the overall widget
        self.status_widget = QWidget()
        self.status_layout = qt.QVBoxLayout()

        #preliminaries to build the rest of the panel
        num_robots = len(self.ros_gui_bridge.robot_names) #get the number of robots
        statuses = ['Battery(mins)', 'Comms', 'Mobility', 'Camera', 'Velodyne', 'CPU', 'Disk Space'] #define the status each robot will have

        status_label = qt.QLabel()
        status_label.setText('STATUS PANEL')
        status_label.setAlignment(Qt.AlignCenter)
        self.status_layout.addWidget(status_label)

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

        #add fake data for each robot
        self.status_table.setItem(0,0, qt.QTableWidgetItem(''))
        self.status_table.setItem(0,1, qt.QTableWidgetItem(''))

        self.status_table.setItem(1,0, qt.QTableWidgetItem(''))
        self.status_table.setItem(1,1, qt.QTableWidgetItem(''))

        self.status_table.setItem(2,0, qt.QTableWidgetItem(''))
        self.status_table.setItem(2,1, qt.QTableWidgetItem(''))

        self.status_table.setItem(3,0, qt.QTableWidgetItem(''))
        self.status_table.setItem(3,1, qt.QTableWidgetItem(''))

        self.status_table.setItem(4,0, qt.QTableWidgetItem(''))
        self.status_table.setItem(4,1, qt.QTableWidgetItem(''))

        self.status_table.setItem(5,0, qt.QTableWidgetItem(''))
        self.status_table.setItem(5,1, qt.QTableWidgetItem(''))

        self.status_table.setItem(6,0, qt.QTableWidgetItem(''))
        self.status_table.setItem(6,1, qt.QTableWidgetItem(''))


        #color the squares
        # self.status_table.item(0,0).setBackground(gui.QColor(220,0,0))
        # self.status_table.item(0,1).setBackground(gui.QColor(255,165,0))

        # self.status_table.item(1,0).setBackground(gui.QColor(0,220,0))
        # self.status_table.item(1,1).setBackground(gui.QColor(255,165,0))

        # self.status_table.item(2,0).setBackground(gui.QColor(0,220,0))
        # self.status_table.item(2,1).setBackground(gui.QColor(220,0,0))

        # self.status_table.item(3,0).setBackground(gui.QColor(0,220,0))
        # self.status_table.item(3,1).setBackground(gui.QColor(0,220,0))

        # self.status_table.item(4,0).setBackground(gui.QColor(0,220,0))
        # self.status_table.item(4,1).setBackground(gui.QColor(255,165,0))

        # self.status_table.item(5,0).setBackground(gui.QColor(220,0,0))
        # self.status_table.item(5,1).setBackground(gui.QColor(0,220,0))

        # self.status_table.item(6,0).setBackground(gui.QColor(220,0,0))
        # self.status_table.item(6,1).setBackground(gui.QColor(0,220,0))

        #add the table to the layout
        self.status_layout.addWidget(self.status_table)

        #add to the overall gui
        self.status_widget.setLayout(self.status_layout)
        self.global_widget.addWidget(self.status_widget, pos[0], pos[1], pos[2], pos[3])

    def queueClick(self, row, col):
        '''
        Callback that is run when something in the artifact
        queue is selected
        '''

        #remove the "unread" indicator if its there
        with self.update_queue_lock:
            self.queue_table.setItem(row,4, qt.QTableWidgetItem(str('')))
            self.queue_table.item(row, 4)#.setBackground(gui.QColor(0,255,0))#color the unread green

            clicked_robot_id =  int(self.queue_table.item(row, 0).text())
            priority = self.queue_table.item(row, 1).text()
            category = self.queue_table.item(row, 3).text()
            artifact_report_id  = self.queue_table.item(row, 5).text()

            #highlight the entire row
            self.queue_table.selectRow(row)



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

            if (int(art.source_robot) == int(clicked_robot_id)) and\
               (int(art.artifact_report_id) == int(artifact_report_id)):
                
                artifact = art

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

        
    def updateArtifactInQueue(self, artifact):
        '''
        An artifact's info has been changed and the queue needs to be updated
        '''

        for i in range(self.queue_table.rowCount()):
            robot_id = self.queue_table.item(i, 0).text()
            art_id = self.queue_table.item(i, 5).text()

            if (int(artifact.source_robot) == int(robot_id)) and \
               (int(artifact.artifact_report_id) == int(art_id)):

                row = i

                #to refresh the data
                self.queue_table.removeRow(row)
                self.queue_table.insertRow(row)
                # print "found: ",self.queue_table.item(row, 0).text(), self.queue_table.item(row, 1).text(), self.queue_table.item(row, 2).text()  

                #update the necessary info
                if (artifact.time_from_robot == -1): #this is coming directly from the robot
                    disp_time = self.darpa_gui_bridge.displaySeconds(float(self.darpa_gui_bridge.darpa_status_update['run_clock']))
                
                else:
                    disp_time = self.darpa_gui_bridge.displaySeconds(float(artifact.time_from_robot))


                row_data = [artifact.source_robot, artifact.priority, disp_time, \
                            artifact.category, 'Updtd', artifact.artifact_report_id]


                with self. update_queue_lock:

                    self.queue_table.setSortingEnabled(False) #to avoid corrupting the table

                    for col, val in enumerate(row_data):
                        
                        if (str(val).find(':')==-1): #if we're not dealing with a display time
                            item = NumericItem(str(val))
                            item.setData(core.Qt.UserRole, val)
                            
                        
                        else:
                            colon = disp_time.find(':')
                            val = float(disp_time[:colon])*60 + float(disp_time[colon+1:])
                            item = NumericItem(str(disp_time))
                            item.setData(core.Qt.UserRole, val)

                        self.queue_table.setItem(row, col, item)
                        # self.queue_table.itemChanged(item)#self.queue_table.item(row, col))


                    #color the unread green
                    self.queue_table.item(row, 4).setBackground(gui.QColor(0,255,0))

                    for k in range(self.queue_table.columnCount()): #make the cells not editable and make the text centered
                        if self.queue_table.item(row, k) != None: 
                            self.queue_table.item(row, k).setFlags( core.Qt.ItemIsSelectable |  core.Qt.ItemIsEnabled )
                            self.queue_table.item(row, k).setTextAlignment(Qt.AlignHCenter) 

                    self.queue_table.setSortingEnabled(True) #to avoid corrupting the table

                    if(self.queue_table_sort_button.isChecked()): #if the sort button is pressed, sort the incoming artifacts
                        self.queue_table.sortItems(2, core.Qt.DescendingOrder)
        



    def removeQueueItem(self, row):
        '''
        Function to remove an item from the queue
        '''
        self.queue_table.removeRow(row)


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
        self.queue_layout.addWidget(queue_label, 0, 0)

        #add the sort on/off button
        self.queue_table_sort_button = qt.QPushButton("Sort by time")
        self.queue_table_sort_button.setCheckable(True) # a button pressed will stay pressed, until unclicked
        self.queue_table_sort_button.toggle() #start with it sorting the table

        self.queue_layout.addWidget(self.queue_table_sort_button, 1, 0)

         #make a table
        self.queue_table = qt.QTableWidget()

        #resize the cells to fill the widget 
        self.queue_table.horizontalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)
        # self.queue_table.verticalHeader().setSectionResizeMode(qt.QHeaderView.ResizeToContents)
        # self.queue_table.resizeRowsToContents() #to enable word wrapping on the categories

        self.queue_table.setColumnCount(6) # set column count        
        self.queue_table.setHorizontalHeaderLabels(['Robot\nNum', 'Priority', 'Detect\nTime', '   Category   ', 'Unread', 'Artifact Report ID']) #make the column headers

        #hide the artifact
        self.queue_table.setColumnHidden(5,True)

        #resize the column heading depending on the content
        # header = self.queue_table.horizontalHeader()
        # header.setSectionResizeMode(0, qt.QHeaderView.ResizeToContents)
        # header.setSectionResizeMode(1, qt.QHeaderView.ResizeToContents)
        # header.setSectionResizeMode(2, qt.QHeaderView.ResizeToContents)
        # header.setSectionResizeMode(3, qt.QHeaderView.Stretch)
        # header.setSectionResizeMode(4, qt.QHeaderView.ResizeToContents)


        #make sortable
        self.queue_table.setSortingEnabled(True)


        #add click listener
        self.queue_table.cellDoubleClicked.connect(self.queueClick)


        #add the table to the layout
        self.queue_layout.addWidget(self.queue_table)

        #add to the overall gui
        self.queue_widget.setLayout(self.queue_layout)
        self.global_widget.addWidget(self.queue_widget, pos[0], pos[1], pos[2], pos[3]) #last 2 parameters are rowspan and columnspan

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
        self.info_label.setText('Time Left: -- \t Score: -- \t Proposals Left: --')
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
      





    def buildGui(self):
        '''
        Function to layout the gui (place widgets, etc.)
        '''

        # self.init_buttons(self.config_filename)

        #define the position of everything in terms of row, column
        info_pos    = [0,1,1,1]
        bigred_pos  = [0,2]
        status_pos  = [1,2,4,1]
        queue_pos   = [1,0,4,1] #last 2 parameters are rowspan and columnspan
        control_pos = [5,2,2,1]
        artvis_pos  = [1,1,4,1]
        arthist_pos = [5,0,2,1]     
        artmanip_pos =[5,1,1,1]   
        

        #initialize the panels
        self.initControlPanel(control_pos) #the panel to send robots commands (e-stop, etc.)
        self.initArtifactVisualizer(artvis_pos) #panel to display and react to artifacts as they come in
        self.initStatusPanel(status_pos) #panel to display health info of each robot
        self.initArtifactQueue(queue_pos) #panel to display the artifact queue
        self.initBigRed(bigred_pos) #the big red button to e-stop all of the robots
        self.initInfoPanel(info_pos) #table to display information (score, artifacts proposed, etc.) about our run
        self.initArtHistoryPanel(arthist_pos) #table to display artifact proposals
        self.initArtifactManipulator(artmanip_pos) #panel to complete actions w.r.t. artifacts

        #initialize the subscribers for updating different parts of the GUI
        self.info_subscriber = rospy.Subscriber('/darpa_status_updates', String, self.updateInfoPanel)

    def updateInfoPanel(self, msg):
        '''
        Subscriber that constantly updates info panel
        '''

        
        #update the info panel
        info_update = self.darpa_gui_bridge.darpa_status_update

        self.info_label.setText(msg.data)

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

        #all we really need is the last line
        csv_data = None
        with open(filename, 'r') as f:
            for row in reversed(list(csv.reader(f))):
                csv_data = row
                break

        if (csv_data == None):
            print "There was nothing in the csv file!!"

        else:
            #parse the artifacts
            artifact_data = csv_data[1]
            split_art_data = artifact_data.split('//')

            for art_str in split_art_data:

                split_artifact = art_str.split('|')

                if (len(split_artifact)>9): #if we actually have something here

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


                    
                    if (artifact.darpa_response != None): #this artifact has been submitted to darpa

                        with self.artifact_proposal_lock:

                            self.arthist_table.setSortingEnabled(False) 

                            #add to the engine
                            self.gui_engine.all_artifacts.append(artifact)
                            self.gui_engine.submitted_artifacts.append(artifact)

                            #add to the submission panel
                            submission_time = self.darpa_gui_bridge.displaySeconds(artifact.time_to_darpa)

                            submission_correct = artifact.darpa_response[artifact.darpa_response.find('?'):]
                            
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
                            
                                if (str(val).find(':')==-1): #if we're not dealing with a display time
                                    item = NumericItem(str(val))
                                    item.setData(core.Qt.UserRole, val)
                                    
                                
                                else:
                                    colon = submission_time.find(':')
                                    val = float(submission_time[:colon])*60 + float(submission_time[colon+1:])
                                    item = NumericItem(str(submission_time))
                                    item.setData(core.Qt.UserRole, val)

                                self.arthist_table.setItem(row, col, item)

                            #add the response item, we don't want this to be a NumericItem
                            self.arthist_table.setItem(row, 3, response_item)

                            self.arthist_table.setSortingEnabled(True) 

                            if(self.arthist_table_sort_button.isChecked()): #if the sort button is pressed, sort the incoming artifacts
                                    self.arthist_table.sortItems(1, core.Qt.DescendingOrder)



                    else: #this artifact is still in the queue

                        #add the artifact to the list of queued objects and to the all_artifacts list
                        self.gui_engine.queued_artifacts.append(artifact)
                        self.gui_engine.all_artifacts.append(artifact)

                        #call a function to graphically add it to the queue
                        self.sendToQueue(artifact)



            #parse the vehicle states
            vehicle_data = csv_data[2] 

            vehicle_data_split = vehicle_data.split('|')

            for i, command in enumerate(vehicle_data_split):

                if (len(command)>1): #we actually have something here

                    #go find the button this corresponds to and "press" it
                    found_button = False

                    for button in self.control_buttons[i]:
                        if(button.text() == command):
                            found_button = True
                            button.click()

                    if(found_button != True):
                        print "Could not find the appropriate command button to be pressed"


            #parse the info
            info_data = csv_data[3] 


            #save periodically
            self.gui_engine.savePeriodically(self)

            print "Data loaded from csv"







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
        

    def initiateSettings(self, config_filename):
        '''
        Generates the gui using either start fresh or from using previous settings
        '''
        self.config_filename = config_filename
        self.ros_gui_bridge = RosGuiBridge(self.config_filename, self)
        self.darpa_gui_bridge = DarpaGuiBridge(self.config_filename)
        self.gui_engine = GuiEngine(self)
        
        self.buildGui()



    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog






