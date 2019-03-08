#!/usr/bin/python
'''
File containing all things related to the frontend of the gui (buttons, colors, panel positions, etc.)
Contact: Bob DeBortoli (debortor@oregonstate.edu)
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
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy

from GuiBridges import RosGuiBridge, DarpaGuiBridge

from functools import partial

import pdb

from argparse import ArgumentParser

from GuiEngine import GuiEngine





class BasestationGuiPlugin(Plugin):
    def __init__(self, context):
        super(BasestationGuiPlugin, self).__init__(context)
        self.setObjectName('BasestationGuiPlugin')

        #if we're also simulating the darpa command post
        self.simulating_command_post = rospy.get_param("/simulating_command_post")

        
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
        self.top_layout.addWidget(self.config_button)

        self.state_label = qt.QLabel('state: ')
        self.top_layout.addWidget(self.state_label)


        self.global_widget.addWidget(self.top_widget)
        
        
        self.widget.setLayout(self.global_widget)
        context.add_widget(self.widget)

        self.artifact_proposal_lock = threading.Lock()
        self.update_queue_lock = threading.Lock()

        self.displayed_artifact = None #which artifact is currently being displayed


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
                button.clicked.connect(partial(self.ros_gui_bridge.publishRobotCommand, command, robot_name, button))

                robot_layout.addWidget(button, row, col)

                row+=1

            self.control_buttons.append(robot_button_list)



            #add the layout to a groupbox
            robot_groupbox.setLayout(robot_layout)

            self.control_layout.addWidget(robot_groupbox)

        #add the control panel to the overall gui
        self.control_widget.setLayout(self.control_layout)
        self.global_widget.addWidget(self.control_widget, pos[0], pos[1], pos[2], pos[3])


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
        self.art_image.setText('thing\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n')
        self.art_image.setStyleSheet('background: black')
        # self.art_image.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.artvis_layout.addWidget(self.art_image, 1, 0, 1, 4) #last 2 parameters are rowspan and columnspan

        

        #add to the overall gui
        self.artvis_widget.setLayout(self.artvis_layout)
        self.global_widget.addWidget(self.artvis_widget, pos[0], pos[1], pos[2], pos[3])

    def initArtifactManipulator(self, pos):
        '''
        Buttons and textboxes to refine the position and complete actions with the  artifact
        '''

         #define the overall widget
        self.artmanip_widget = QWidget()
        self.artmanip_layout = qt.QGridLayout()

        #add in information about 3d position     
        dimensions = ['X', 'Y', 'Z'] 

        boldFont = gui.QFont()
        boldFont.setBold(True)

        for i,dim in enumerate(dimensions):
            dim_label = qt.QLabel()
            dim_label.setText(dim)
            dim_label.setAlignment(Qt.AlignCenter)
            dim_label.setFont(boldFont)
            self.artmanip_layout.addWidget(dim_label, 0, i+1)

        #information about the detected position
        robot_pos = ['N/A', 'N/A', 'N/A'] #fake data

        self.orig_pos_label = qt.QLabel()
        self.orig_pos_label.setText('Original Position')
        self.artmanip_layout.addWidget(self.orig_pos_label, 1, 0)

        self.orig_pos_label_x = qt.QLabel()
        self.orig_pos_label_x.setText(str(robot_pos[0]))
        self.orig_pos_label_x.setAlignment(Qt.AlignCenter)
        self.artmanip_layout.addWidget(self.orig_pos_label_x, 1, 1)

        self.orig_pos_label_y = qt.QLabel()
        self.orig_pos_label_y.setText(str(robot_pos[1]))
        self.orig_pos_label_y.setAlignment(Qt.AlignCenter)
        self.artmanip_layout.addWidget(self.orig_pos_label_y, 1, 2)

        self.orig_pos_label_z = qt.QLabel()
        self.orig_pos_label_z.setText(str(robot_pos[2]))
        self.orig_pos_label_z.setAlignment(Qt.AlignCenter)
        self.artmanip_layout.addWidget(self.orig_pos_label_z, 1, 3)

        #editable information about the position, to send to darpa
        refined_pos_label = qt.QLabel()
        refined_pos_label.setText('Refined Position')
        self.artmanip_layout.addWidget(refined_pos_label, 2, 0)

        self.art_pos_textbox_x, self.art_pos_textbox_y, self.art_pos_textbox_z = qt.QLineEdit(), qt.QLineEdit(), qt.QLineEdit()
       
        #fill in some fake data
        self.art_pos_textbox_x.setText(str(robot_pos[0]))
        self.artmanip_layout.addWidget(self.art_pos_textbox_x, 2, 1)

        self.art_pos_textbox_y.setText(str(robot_pos[1]))
        self.artmanip_layout.addWidget(self.art_pos_textbox_y, 2, 2)

        self.art_pos_textbox_z.setText(str(robot_pos[2]))
        self.artmanip_layout.addWidget(self.art_pos_textbox_z, 2, 3)





        #add in a few buttons at the bottom to do various things
        self.artmanip_button_layout = qt.QHBoxLayout()
        self.artmanip_button_list = []

        #add the buttons and textboxes at the bottom
        art_action_label1 = qt.QLabel()
        art_action_label1.setText('\n\nAction')
        art_action_label1.setAlignment(Qt.AlignCenter)
        self.artmanip_layout.addWidget(art_action_label1, 3, 0, 1, 2)

        art_action_label2 = qt.QLabel()
        art_action_label2.setText('\n\nCategory')
        art_action_label2.setAlignment(Qt.AlignCenter)
        self.artmanip_layout.addWidget(art_action_label2, 3, 2)

        art_action_label3 = qt.QLabel()
        art_action_label3.setText('\n\nPriority')
        art_action_label3.setAlignment(Qt.AlignCenter)
        self.artmanip_layout.addWidget(art_action_label3, 3, 3)


        button = qt.QPushButton("    To DARPA    ")
        button.clicked.connect(partial(self.proposeArtifact))
        self.artmanip_layout.addWidget(button, 4, 0, 1, 2)

        self.darpa_cat_box = qt.QComboBox() #textbox to manually fill in the category for submission to darpa

        for category in self.ros_gui_bridge.artifact_categories:
            self.darpa_cat_box.addItem(category)

        self.artmanip_layout.addWidget(self.darpa_cat_box, 4,2)

        self.darpa_cat_box.currentTextChanged.connect(self.updateArtifactCat)

        # button = qt.QPushButton("    To Queue    ")
        # button.clicked.connect(partial(self.sendToQueue))
        # self.artmanip_layout.addWidget(button, 5, 0, 1, 2)

        # self.queue_cat_box = qt.QComboBox()

        # for category in self.ros_gui_bridge.artifact_categories:
        #     self.queue_cat_box.addItem(category)

        #set the defauly value of the artifact to whats being displayed
         #variables for storing the current information about the artifact being examined
        self.artifact_cat = self.ros_gui_bridge.artifact_categories[0]
        self.artifact_id = 0

        # self.artmanip_layout.addWidget(self.queue_cat_box, 5,2)


        # self.queue_cat_box.currentTextChanged.connect(self.updateArtifactCat)

        self.queue_priority_box = qt.QComboBox() #way to fill in the priority
        self.queue_priority_box.addItem("    1    ")
        self.artifact_priority = "    1    "
        self.queue_priority_box.addItem("    2    ")
        self.queue_priority_box.addItem("    3    ")
        self.queue_priority_box.addItem("    4    ")
        # self.artmanip_layout.addWidget(self.queue_priority_box, 5,3)

        # self.queue_priority_box.currentTextChanged.connect(self.updateArtifactPriority)

        button = qt.QPushButton("    Discard    ")
        # button.setSizePolicy(QSizePolicy.Expanding, 0)
        self.artmanip_layout.addWidget(button, 6, 0, 1, 2)

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
        if(self.simulating_command_post):
            with self.artifact_proposal_lock: #to ensure we only draw one response at once
            
                data = [ float(self.art_pos_textbox_x.text()), float(self.art_pos_textbox_y.text()), \
                         float(self.art_pos_textbox_z.text()), self.darpa_cat_box.currentText()]

                
                proposal_return = self.darpa_gui_bridge.startArtifactProposal(data)

                if(len(proposal_return) > 0):
                    [submission_time, artifact_type, x, y, z, report_status, score_change, http_response, http_reason] = \
                                                                                                    proposal_return

                    #add this to the submission history panel 
                    submission_time = self.displaySeconds(submission_time)

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

                    self.arthist_table.insertRow(self.arthist_table.rowCount())

                    self.arthist_table.setItem(self.arthist_table.rowCount() - 1, 0, qt.QTableWidgetItem(str(artifact_type)))
                    self.arthist_table.setItem(self.arthist_table.rowCount() - 1, 1, qt.QTableWidgetItem(str(submission_time)))
                    self.arthist_table.setItem(self.arthist_table.rowCount() - 1, 2, qt.QTableWidgetItem(str(int(x))+'/'+str(int(y))+'/'+str(int(z))))
                    self.arthist_table.setItem(self.arthist_table.rowCount() - 1, 3, response_item)



                    #go find the artifact in the queue and remove it
                    if(self.displayed_artifact!=None): #if we have actually set the artifact
                        for i in range(self.queue_table.rowCount()):
                            robot_id = self.queue_table.item(i, 0).text()
                            artifact_id = self.queue_table.item(i, 1).text()

                            if (int(self.displayed_artifact.source_robot) == int(robot_id)) and \
                               (int(self.displayed_artifact.artifact_report_id) == int(artifact_id)):

                                self.queue_table.removeRow(self.queue_table.item(i,0).row())

                                #also remove it from the gui engine artifacts list and put it into the gui engine proposed list
                                self.gui_engine.queued_artifacts.remove(self.displayed_artifact)
                                self.gui_engine.submitted_artifacts.append(self.displayed_artifact)


                                break


                    #remove the artifact from the main visualization panel
                    self.orig_pos_label_x.setText('')
                    self.art_pos_textbox_x.setText('')

                    self.orig_pos_label_y.setText('')
                    self.art_pos_textbox_y.setText('')

                    self.orig_pos_label_z.setText('')
                    self.art_pos_textbox_z.setText('')

                    self.displayed_artifact = None




                


    def removeArtifactFromQueue(self,artifact):
        '''
        Remove an artifact from the artifact queue
        '''


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
        '''
        self.artifact_priority = str(self.queue_priority_box.currentText())

    def updateArtifactCat(self):
        '''
        The combo box for changing the artifact category was pressed
        '''
        # self.artifact_cat = str(self.queue_cat_box.currentText())
        pass


    def sendToQueue(self, artifact):
        '''
        Send the artifact being examined to the queue
        '''

        # self.displaySeconds(self.darpa_gui_bridge.darpa_status_update['run_clock'])
        # print 

        with self.update_queue_lock:
            self.queue_table.insertRow(self.queue_table.rowCount())
            self.queue_table.setItem(self.queue_table.rowCount() - 1, 0, qt.QTableWidgetItem(str(artifact.source_robot)))
            self.queue_table.setItem(self.queue_table.rowCount() - 1, 1, qt.QTableWidgetItem(str(artifact.artifact_report_id)))
            # self.queue_table.setItem(self.queue_table.rowCount() - 1, 2, qt.QTableWidgetItem(str(self.displaySeconds(self.self.darpa_gui_bridge.darpa_status_update['run_clock']))))
            self.queue_table.setItem(self.queue_table.rowCount() - 1, 2, qt.QTableWidgetItem(str(self.displaySeconds(random.random()*100.))))
            self.queue_table.setItem(self.queue_table.rowCount() - 1, 3, qt.QTableWidgetItem(str(artifact.category)))
            self.queue_table.setItem(self.queue_table.rowCount() - 1, 4, qt.QTableWidgetItem(str('   !   ')))

            #color the unread green
            self.queue_table.item(self.queue_table.rowCount() - 1, 4).setBackground(gui.QColor(0,255,0))

            for i in range(5): #make the cells not editable
                self.queue_table.item(self.queue_table.rowCount() - 1, i).setFlags( core.Qt.ItemIsSelectable |  core.Qt.ItemIsEnabled )


        #############
        #below this: old code for adding to the queue
        #############

        # self.queue_table.insertRow(self.queue_table.rowCount())

        # self.queue_table.setItem(self.queue_table.rowCount() - 1, 0, qt.QTableWidgetItem(str(self.artifact_id)))
        # self.queue_table.setItem(self.queue_table.rowCount() - 1, 1, qt.QTableWidgetItem(str(self.artifact_cat)))
        # self.queue_table.setItem(self.queue_table.rowCount() - 1, 2, qt.QTableWidgetItem(str(self.artifact_priority)))


        # #increase the artifact id so that the next artifact has a different id
        # self.artifact_id+=1

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
        self.status_table.setItem(0,0, qt.QTableWidgetItem('15'))
        self.status_table.setItem(0,1, qt.QTableWidgetItem('44'))

        self.status_table.setItem(1,0, qt.QTableWidgetItem('In-range'))
        self.status_table.setItem(1,1, qt.QTableWidgetItem('Near-limits'))

        self.status_table.setItem(2,0, qt.QTableWidgetItem('Moving'))
        self.status_table.setItem(2,1, qt.QTableWidgetItem('Stuck'))

        self.status_table.setItem(3,0, qt.QTableWidgetItem('Ok'))
        self.status_table.setItem(3,1, qt.QTableWidgetItem('Ok'))

        self.status_table.setItem(4,0, qt.QTableWidgetItem('Ok'))
        self.status_table.setItem(4,1, qt.QTableWidgetItem('Warning'))

        self.status_table.setItem(5,0, qt.QTableWidgetItem('90%'))
        self.status_table.setItem(5,1, qt.QTableWidgetItem('40%'))

        self.status_table.setItem(6,0, qt.QTableWidgetItem('90%'))
        self.status_table.setItem(6,1, qt.QTableWidgetItem('40%'))


        #color the squares
        self.status_table.item(0,0).setBackground(gui.QColor(220,0,0))
        self.status_table.item(0,1).setBackground(gui.QColor(255,165,0))

        self.status_table.item(1,0).setBackground(gui.QColor(0,220,0))
        self.status_table.item(1,1).setBackground(gui.QColor(255,165,0))

        self.status_table.item(2,0).setBackground(gui.QColor(0,220,0))
        self.status_table.item(2,1).setBackground(gui.QColor(220,0,0))

        self.status_table.item(3,0).setBackground(gui.QColor(0,220,0))
        self.status_table.item(3,1).setBackground(gui.QColor(0,220,0))

        self.status_table.item(4,0).setBackground(gui.QColor(0,220,0))
        self.status_table.item(4,1).setBackground(gui.QColor(255,165,0))

        self.status_table.item(5,0).setBackground(gui.QColor(220,0,0))
        self.status_table.item(5,1).setBackground(gui.QColor(0,220,0))

        self.status_table.item(6,0).setBackground(gui.QColor(220,0,0))
        self.status_table.item(6,1).setBackground(gui.QColor(0,220,0))

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
        # print "here"

        #remove the "unread" indicator if its there
        self.queue_table.setItem(row,4, qt.QTableWidgetItem(str('')))
        self.queue_table.item(row, 4)#.setBackground(gui.QColor(0,255,0))#color the unread green
        # self.displayed_artifact.unread = False


        clicked_robot_id =  int(self.queue_table.item(row, 0).text())
        clicked_id =  self.queue_table.item(row, 1).text()

        #change the combo boxes
        index = self.darpa_cat_box.findText(self.queue_table.item(row, 3).text(), core.Qt.MatchFixedString)
        if index >= 0:
             self.darpa_cat_box.setCurrentIndex(index)

        #associate with a specific artifact
        #todo: put in some error checking
        for art in self.gui_engine.queued_artifacts:
            if (int(art.source_robot) == int(clicked_robot_id)) and\
               (int(art.artifact_report_id) == int(clicked_id)):
                artifact = art
                break


        #change the text of the xyz positions
        robot_pos = artifact.pos

        #fill in the positional data
        self.orig_pos_label_x.setText(str(robot_pos[0]))
        self.orig_pos_label_y.setText(str(robot_pos[1]))
        self.orig_pos_label_z.setText(str(robot_pos[2]))
        
        self.art_pos_textbox_x.setText(str(robot_pos[0]))
        self.art_pos_textbox_y.setText(str(robot_pos[1]))
        self.art_pos_textbox_z.setText(str(robot_pos[2]))


        #update the global info for what artifact is being displayed
        self.updateDisplayedArtifact(artifact)

        if(self.displayed_artifact!=None):
            self.displayed_artifact.unread = False

        #highlight the entire row
        self.queue_table.selectRow(row)



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
        self.queue_layout = qt.QVBoxLayout()

        queue_label = qt.QLabel()
        queue_label.setText('ARTIFACT QUEUE')
        queue_label.setAlignment(Qt.AlignCenter)
        self.queue_layout.addWidget(queue_label)

         #make a table
        self.queue_table = qt.QTableWidget()
        # self.queue_table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        #resize the cells to fill the widget 
        self.queue_table.horizontalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)

        self.queue_table.setColumnCount(5) # set column count        
        self.queue_table.setHorizontalHeaderLabels(['Robot\nNum', 'Art.\nID', 'Detected\nTime', '   Category   ', 'Unread?']) #make the column headers

        # header = self.queue_table.horizontalHeader()
        
        # header.setSectionResizeMode(0, qt.QHeaderView.ResizeToContents)
        # header.setSectionResizeMode(1, qt.QHeaderView.ResizeToContents)
        # header.setSectionResizeMode(4, qt.QHeaderView.Stretch)

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

        button = qt.QPushButton("ESTOP ALL\n  ROBOTS")
        button.setStyleSheet("background-color: red")
        self.bigred_layout.addWidget(button)            

        #add to the overall gui
        self.bigred_widget.setLayout(self.bigred_layout)
        self.global_widget.addWidget(self.bigred_widget, pos[0], pos[1])


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
        self.arthist_layout.addWidget(arthist_label)

         #make a table
        self.arthist_table = qt.QTableWidget()


        #resize the cells to fill the widget 
        self.arthist_table.horizontalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)
        # self.arthist_table.verticalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)

        self.arthist_table.setColumnCount(4) # set column count        
        self.arthist_table.setHorizontalHeaderLabels(['Category', 'Time', 'x/y/z', 'Response']) #make the column headers

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
        artmanip_pos = [5,1,1,1]   
        

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



    def select_config_file(self):
        starting_path = os.path.join(rospkg.RosPack().get_path('basestation_gui_python'), 'config')
        filename = qt.QFileDialog.getOpenFileName(self.widget, 'Open Config File', starting_path, "Config Files (*.yaml)")[0]
        if filename != '':
            self.config_filename = filename
            self.ros_gui_bridge = RosGuiBridge(self.config_filename)
            self.darpa_gui_bridge = DarpaGuiBridge(self.config_filename)
            self.buildGui()
            

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
        if(self.simulating_command_post):
            self.darpa_gui_bridge.shutdownHttpServer()
        

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('config_filename', self.config_filename)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        '''
        Function which lays out all of the widgets from a pre-specified .yaml 
        config file. 
        '''
        self.config_filename = instance_settings.value('config_filename')
        self.ros_gui_bridge = RosGuiBridge(self.config_filename)
        self.darpa_gui_bridge = DarpaGuiBridge(self.config_filename)
        self.gui_engine = GuiEngine(self)
        self.buildGui()
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog






