import os
import time
import rospy
import rospkg
from std_msgs.msg import String
import numpy as np
import yaml
import collections

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





class BasestationGuiPlugin(Plugin):
    def __init__(self, context):
        super(BasestationGuiPlugin, self).__init__(context)
        self.setObjectName('BasestationGuiPlugin')
        
        self.state_sub = rospy.Subscriber('/state', String, self.state_callback)
        self.transition_pub = rospy.Publisher('/transition', String, queue_size=1)
        self.config_filename = ''
        
        self.widget = QWidget()
        self.global_widget = qt.QGridLayout()

        self.top_widget = qt.QWidget()
        self.top_layout = qt.QHBoxLayout()
        self.top_widget.setLayout(self.top_layout)

        self.config_button = qt.QPushButton('Open Config...')
        self.config_button.clicked.connect(self.select_config_file)
        self.top_layout.addWidget(self.config_button)

        self.state_label = qt.QLabel('state: ')
        self.top_layout.addWidget(self.state_label)

        self.global_widget.addWidget(self.top_widget)
        
        self.button_widget = qt.QWidget()
        self.button_layout = qt.QHBoxLayout()
        self.button_widget.setLayout(self.button_layout)

        #config_filename = os.path.join(rospkg.RosPack().get_path('state_machine'), 'config', 'state_machine.yaml')
        #self.init_buttons(config_filename)
        
        self.global_widget.addWidget(self.button_widget)
        
        self.widget.setLayout(self.global_widget)
        context.add_widget(self.widget)

       


    def init_buttons(self, config_filename):
        self.buttons = {}

        for i in reversed(range(self.button_layout.count())):
            self.button_layout.itemAt(i).widget().setParent(None)
        
        for transition in self.ros_gui_bridge.get_transitions():
            def get_publish_transition_function(t):
                def publish_transition():
                    msg = String()
                    msg.data = t
                    self.transition_pub.publish(msg)
                return publish_transition
            button = qt.QPushButton(transition)
            button.clicked.connect(get_publish_transition_function(transition))
            self.buttons[transition] = button
            self.button_layout.addWidget(button)

    def init_control_panel(self, pos):
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
                robot_button_list.append(button)
                
                #upon press, change the button color
                button.clicked.connect(partial(self.ros_gui_bridge.changeControlButtonColors, self.control_buttons, robot_num, button))
                
                #upon press, do something in ROS
                button.clicked.connect(partial(self.ros_gui_bridge.publishRobotCommand, command, robot_name))

                robot_layout.addWidget(button, row, col)

                row+=1

            self.control_buttons.append(robot_button_list)



            #add the layout to a groupbox
            robot_groupbox.setLayout(robot_layout)

            self.control_layout.addWidget(robot_groupbox)

        #add the control panel to the overall gui
        self.control_widget.setLayout(self.control_layout)
        self.global_widget.addWidget(self.control_widget, pos[0], pos[1], pos[2], pos[3])


    def init_artifact_visualizer(self, pos):
        '''
        Panel to visualize the artifacts and do something (send to DARPA),
        add to queue, etc.
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
        self.art_image.setText('thing\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n')
        self.art_image.setStyleSheet('background: black')
        self.art_image.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.artvis_layout.addWidget(self.art_image, 1, 0, 1, 4) #last 2 parameters are rowspan and columnspan

        #add in information about 3d position     
        dimensions = ['X', 'Y', 'Z'] 

        boldFont = gui.QFont()
        boldFont.setBold(True)

        for i,dim in enumerate(dimensions):
            dim_label = qt.QLabel()
            dim_label.setText(dim)
            dim_label.setAlignment(Qt.AlignCenter)
            dim_label.setFont(boldFont)
            self.artvis_layout.addWidget(dim_label, 2, i+1)

        #information about the detected position
        robot_pos = [110.002, 112.000002, 5.000002] #fake data

        orig_pos_label = qt.QLabel()
        orig_pos_label.setText('Original Position')
        self.artvis_layout.addWidget(orig_pos_label, 3, 0)

        for i, orig_pos in enumerate(robot_pos):
            orig_pos_label = qt.QLabel()
            orig_pos_label.setText(str(orig_pos))
            orig_pos_label.setAlignment(Qt.AlignCenter)
            self.artvis_layout.addWidget(orig_pos_label, 3, i+1)

        #editable information about the position, to send to darpa
        refined_pos_label = qt.QLabel()
        refined_pos_label.setText('Refined Position')
        self.artvis_layout.addWidget(refined_pos_label, 4, 0)

        self.art_pos_textbox_x, self.art_pos_textbox_y, self.art_pos_textbox_z = qt.QLineEdit(), qt.QLineEdit(), qt.QLineEdit()
       
        #fill in some fake data
        self.art_pos_textbox_x.setText(str(robot_pos[0]))
        self.artvis_layout.addWidget(self.art_pos_textbox_x, 4, 1)

        self.art_pos_textbox_y.setText(str(robot_pos[1]))
        self.artvis_layout.addWidget(self.art_pos_textbox_y, 4, 2)

        self.art_pos_textbox_z.setText(str(robot_pos[2]))
        self.artvis_layout.addWidget(self.art_pos_textbox_z, 4, 3)



        #add in a few buttons at the bottom to do various things
        self.artvis_button_layout = qt.QHBoxLayout()
        self.artvis_button_list = []

        #add the buttons and textboxes at the bottom
        art_action_label1 = qt.QLabel()
        art_action_label1.setText('\n\nAction')
        art_action_label1.setAlignment(Qt.AlignCenter)
        self.artvis_layout.addWidget(art_action_label1, 5, 0, 1, 2)

        art_action_label2 = qt.QLabel()
        art_action_label2.setText('\n\nCategory')
        art_action_label2.setAlignment(Qt.AlignCenter)
        self.artvis_layout.addWidget(art_action_label2, 5, 2)

        art_action_label3 = qt.QLabel()
        art_action_label3.setText('\n\nPriority')
        art_action_label3.setAlignment(Qt.AlignCenter)
        self.artvis_layout.addWidget(art_action_label3, 5, 3)


        button = qt.QPushButton("    To DARPA    ")
        button.clicked.connect(partial(self.darpa_gui_bridge.sendArtifactProposal,[1.,2.,3.,'human'])) #insert fake human data when calling function
        self.artvis_layout.addWidget(button, 6, 0, 1, 2)

        self.darpa_cat_box = qt.QComboBox() #textbox to manually fill in the category for submission to darpa

        for category in self.ros_gui_bridge.artifact_categories:
            self.darpa_cat_box.addItem(category)

        self.artvis_layout.addWidget(self.darpa_cat_box, 6,2)

        self.darpa_cat_box.currentTextChanged.connect(self.updateArtifactCat)

        button = qt.QPushButton("    To Queue    ")
        button.clicked.connect(partial(self.sendToQueue))
        self.artvis_layout.addWidget(button, 7, 0, 1, 2)

        self.queue_cat_box = qt.QComboBox()

        for category in self.ros_gui_bridge.artifact_categories:
            self.queue_cat_box.addItem(category)

        #set the defauly value of the artifact to whats being displayed
         #variables for storing the current information about the artifact being examined
        self.artifact_cat = self.ros_gui_bridge.artifact_categories[0]
        self.artifact_id = 0

        self.artvis_layout.addWidget(self.queue_cat_box, 7,2)


        self.queue_cat_box.currentTextChanged.connect(self.updateArtifactCat)

        self.queue_priority_box = qt.QComboBox() #way to fill in the priority
        self.queue_priority_box.addItem("    1    ")
        self.artifact_priority = "    1    "
        self.queue_priority_box.addItem("    2    ")
        self.queue_priority_box.addItem("    3    ")
        self.queue_priority_box.addItem("    4    ")
        self.artvis_layout.addWidget(self.queue_priority_box, 7,3)

        self.queue_priority_box.currentTextChanged.connect(self.updateArtifactPriority)

        button = qt.QPushButton("    Discard    ")
        # button.setSizePolicy(QSizePolicy.Expanding, 0)
        self.artvis_layout.addWidget(button, 8, 0, 1, 2)

        #add to the overall gui
        self.artvis_widget.setLayout(self.artvis_layout)
        self.global_widget.addWidget(self.artvis_widget, pos[0], pos[1], pos[2], pos[3])

    def updateArtifactPriority(self):
        '''
        The combo box for changing the artifact priority was pressed
        '''
        self.artifact_priority = str(self.queue_priority_box.currentText())

    def updateArtifactCat(self):
        '''
        The combo box for changing the artifact category was pressed
        '''
        self.artifact_cat = str(self.queue_cat_box.currentText())


    def sendToQueue(self):
        '''
        Send the artifact being examined to the queue
        '''
        
        self.queue_table.insertRow(self.queue_table.rowCount())

        self.queue_table.setItem(self.queue_table.rowCount() - 1, 0, qt.QTableWidgetItem(str(self.artifact_id)))
        self.queue_table.setItem(self.queue_table.rowCount() - 1, 1, qt.QTableWidgetItem(str(self.artifact_cat)))
        self.queue_table.setItem(self.queue_table.rowCount() - 1, 2, qt.QTableWidgetItem(str(self.artifact_priority)))


        #increase the artifact id so that the next artifact has a different id
        self.artifact_id+=1

    def init_status_panel(self, pos):
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
        self.status_table.setItem(0,2, qt.QTableWidgetItem('120'))

        self.status_table.setItem(1,0, qt.QTableWidgetItem('In-range'))
        self.status_table.setItem(1,1, qt.QTableWidgetItem('Near-limits'))
        self.status_table.setItem(1,2, qt.QTableWidgetItem('Out of range'))

        self.status_table.setItem(2,0, qt.QTableWidgetItem('Moving'))
        self.status_table.setItem(2,1, qt.QTableWidgetItem('Stuck'))
        self.status_table.setItem(2,2, qt.QTableWidgetItem('Moving'))

        self.status_table.setItem(3,0, qt.QTableWidgetItem('Ok'))
        self.status_table.setItem(3,1, qt.QTableWidgetItem('Ok'))
        self.status_table.setItem(3,2, qt.QTableWidgetItem('Error'))

        self.status_table.setItem(4,0, qt.QTableWidgetItem('Warning'))
        self.status_table.setItem(4,1, qt.QTableWidgetItem('Ok'))
        self.status_table.setItem(4,2, qt.QTableWidgetItem('Error'))

        self.status_table.setItem(5,0, qt.QTableWidgetItem('90%'))
        self.status_table.setItem(5,1, qt.QTableWidgetItem('40%'))
        self.status_table.setItem(5,2, qt.QTableWidgetItem('45%'))

        self.status_table.setItem(6,0, qt.QTableWidgetItem('90%'))
        self.status_table.setItem(6,1, qt.QTableWidgetItem('40%'))
        self.status_table.setItem(6,2, qt.QTableWidgetItem('45%'))


        #color the squares
        self.status_table.item(0,0).setBackground(gui.QColor(220,0,0))
        self.status_table.item(0,1).setBackground(gui.QColor(255,165,0))
        self.status_table.item(0,2).setBackground(gui.QColor(0,220,0))

        self.status_table.item(1,0).setBackground(gui.QColor(0,220,0))
        self.status_table.item(1,1).setBackground(gui.QColor(255,165,0))
        self.status_table.item(1,2).setBackground(gui.QColor(220,0,0))

        self.status_table.item(2,0).setBackground(gui.QColor(0,220,0))
        self.status_table.item(2,1).setBackground(gui.QColor(220,0,0))
        self.status_table.item(2,2).setBackground(gui.QColor(0,220,0))

        self.status_table.item(3,0).setBackground(gui.QColor(0,220,0))
        self.status_table.item(3,1).setBackground(gui.QColor(0,220,0))
        self.status_table.item(3,2).setBackground(gui.QColor(220,0,0))

        self.status_table.item(4,0).setBackground(gui.QColor(0,220,0))
        self.status_table.item(4,1).setBackground(gui.QColor(255,165,0))
        self.status_table.item(4,2).setBackground(gui.QColor(220,0,0))

        self.status_table.item(5,0).setBackground(gui.QColor(220,0,0))
        self.status_table.item(5,1).setBackground(gui.QColor(0,220,0))
        self.status_table.item(5,2).setBackground(gui.QColor(0,220,0))

        self.status_table.item(6,0).setBackground(gui.QColor(220,0,0))
        self.status_table.item(6,1).setBackground(gui.QColor(0,220,0))
        self.status_table.item(6,2).setBackground(gui.QColor(0,220,0))

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
        self.artifact_id =  int(self.queue_table.item(row, 0).text())
        self.artifact_cat =  self.queue_table.item(row, 1).text()
        self.artifact_priority =  self.queue_table.item(row, 2).text()

        #change the combo boxes
        index = self.queue_cat_box.findText(self.queue_table.item(row, 1).text(), core.Qt.MatchFixedString)
        if index >= 0:
             self.queue_cat_box.setCurrentIndex(index)

        index = self.queue_priority_box.findText(self.queue_table.item(row, 2).text(), core.Qt.MatchFixedString)
        if index >= 0:
             self.queue_priority_box.setCurrentIndex(index)

        #remove the item from the queue
        self.removeQueueItem(row)

    def removeQueueItem(self, row):
        '''
        Function to remove an item from the queue
        '''
        self.queue_table.removeRow(row)


    def init_artifact_queue(self, pos):
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

        #resize the cells to fill the widget 
        self.queue_table.horizontalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)

        self.queue_table.setColumnCount(3) # set column count        
        self.queue_table.setHorizontalHeaderLabels(['ID', 'Category', 'Priority']) #make the column headers

        #make sortable
        self.queue_table.setSortingEnabled(True)


        #add click listener
        self.queue_table.cellDoubleClicked.connect(self.queueClick)


        #add the table to the layout
        self.queue_layout.addWidget(self.queue_table)

        #add to the overall gui
        self.queue_widget.setLayout(self.queue_layout)
        self.global_widget.addWidget(self.queue_widget, pos[0], pos[1], pos[2], pos[3]) #last 2 parameters are rowspan and columnspan

    def init_bigred(self, pos):
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


    def init_info_panel(self, pos):
        '''
        Table to display information (score, artifacts proposed, etc.) about our run
        '''

        #define the overall widget
        self.info_widget = QWidget()
        self.info_layout = qt.QVBoxLayout()

        # info_label = qt.QLabel()
        # info_label.setText('RUN INFO')
        # info_label.setAlignment(Qt.AlignCenter)
        # self.info_layout.addWidget(info_label)

        boldFont = gui.QFont("", 18, gui.QFont.Bold) 


        data_label = qt.QLabel()
        data_label.setText('Time Left: 35:21 \t Score: 10 \t Proposals Left: 11/20')
        data_label.setAlignment(Qt.AlignCenter)
        data_label.setFont(boldFont)

        self.info_layout.addWidget(data_label)

        # info_categories = ['Time Left: ', 'Score: ', 'Proposals Left: ']

        # #make a table
        # self.info_table = qt.QTableWidget()

        # #resize the cells to fill the widget 
        # self.info_table.horizontalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)
        # self.info_table.verticalHeader().setSectionResizeMode(qt.QHeaderView.Stretch)
        
        # self.info_table.setRowCount(1) # set row count
        # self.info_table.setColumnCount(6) # set column count

        # #make the row and column headers
        # self.info_table.setHorizontalHeaderLabels([" "]) 
        # self.info_table.setVerticalHeaderLabels([" "]) 

        # #add fake data for each robot
        # self.info_table.setItem(0,0, qt.QTableWidgetItem(info_categories[0]))
        # self.info_table.setItem(0,1, qt.QTableWidgetItem('35:21'))
        # self.info_table.setItem(0,2, qt.QTableWidgetItem(info_categories[1]))
        # self.info_table.setItem(0,3, qt.QTableWidgetItem('10'))
        # self.info_table.setItem(0,4, qt.QTableWidgetItem(info_categories[2]))
        # self.info_table.setItem(0,5, qt.QTableWidgetItem('11/20'))


        # #add the table to the layout
        # self.info_layout.addWidget(self.info_table)

        #add to the overall gui
        self.info_widget.setLayout(self.info_layout)
        self.global_widget.addWidget(self.info_widget, pos[0], pos[1], pos[2], pos[3])


    def build_gui(self):
        '''
        Function to layout the gui (place widgets, etc.)
        '''

        # self.init_buttons(self.config_filename)

        #define the position of everything in terms of row, column
        info_pos    = [0,1,1,1]
        bigred_pos  = [0,2]
        status_pos  = [1,2,2,1]
        queue_pos   = [1,0,4,1] #last 2 parameters are rowspan and columnspan
        control_pos = [3,2,2,1]
        artvis_pos  = [1,1,4,1]        
        

        #initialize the panels
        self.init_control_panel(control_pos) #the panel to send robots commands (e-stop, etc.)
        self.init_artifact_visualizer(artvis_pos) #panel to display and react to artifacts as they come in
        self.init_status_panel(status_pos) #panel to display health info of each robot
        self.init_artifact_queue(queue_pos) #panel to display the artifact queue
        self.init_bigred(bigred_pos) #the big red button to e-stop all of the robots
        self.init_info_panel(info_pos) #table to display information (score, artifacts proposed, etc.) about our run


    def select_config_file(self):
        starting_path = os.path.join(rospkg.RosPack().get_path('basestation_gui_python'), 'config')
        filename = qt.QFileDialog.getOpenFileName(self.widget, 'Open Config File', starting_path, "Config Files (*.yaml)")[0]
        if filename != '':
            self.config_filename = filename
            self.ros_gui_bridge = RosGuiBridge(self.config_filename)
            self.darpa_gui_bridge = DarpaGuiBridge(self.config_filename)
            self.build_gui()

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
        pass

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('config_filename', self.config_filename)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        '''
        Function which lays out all of the widgets from a pre-specified .yaml 
        config file. 
        '''
        self.config_filename = instance_settings.value('config_filename')
        if(self.config_filename is None):
            #use a hardcoded filename
            starting_path = os.path.join(rospkg.RosPack().get_path('basestation_gui_python'), 'config/gui_params.yaml')

        self.ros_gui_bridge = RosGuiBridge(self.config_filename)
        self.darpa_gui_bridge = DarpaGuiBridge(self.config_filename)
        self.build_gui()
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

