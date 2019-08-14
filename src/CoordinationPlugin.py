import sys, math

import rospy
import rospkg
import tf
import robots
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point32
import threading

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

from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import QPushButton, QVBoxLayout, QHBoxLayout
import signal

from point_to_line_dist import point_to_line_dist

import numpy as np
from shapely import geometry

from functools import partial


'''
see bag file
rosbag play --clock ~/tmp/stix_data/base_station_2019-04-10-11-29-28_0.bag -r 100
( rosparam set /use_sim_time false )
'''


robot_names = []

alpha = 128
robot_colors = [QtGui.QColor(255,0,0,alpha), QtGui.QColor(0,255,0,alpha), QtGui.QColor(0,0,255,alpha),
                QtGui.QColor(255,0,255,alpha), QtGui.QColor(255,255,0,alpha), QtGui.QColor(0,255,255,alpha)]

class CoordinationPlugin(Plugin):
    def __init__(self, context):
        super(CoordinationPlugin, self).__init__(context)
        self.setObjectName('CoordinationPlugin')

        config = robots.Config()
        for r in config.robots:
            robot_names.append(r.topic_prefix)
        
        self.global_widget = GlobalWidget()
        context.add_widget(self.global_widget)

    def shutdown_plugin(self):
        try:
            self.global_widget
        except NameError:
            self.global_widget = None
        if self.global_widget:
            self.global_widget.shutdown()


class GlobalWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        QtWidgets.QWidget.__init__(self, parent)

        self.setWindowTitle("Robot Coordination")
        self.resize(600,500)
        self.layout = QVBoxLayout()
        self.widget_buttons = ButtonsBox(global_widget=self)
        self.widget_buttons.resize(100,200)
        self.layout.addWidget(self.widget_buttons,0)
        self.widget_partition = MapWidget(global_widget=self)
        self.widget_partition.resize(500,500)
        #self.widget_partition.setViewport(50,50,500,500)
        self.layout.addWidget(self.widget_partition,5)

        self.setLayout(self.layout)

    def shutdown(self):
        try:
            self.widget_partition
        except NameError:
            self.widget_partition = None
        if self.widget_partition:
            self.widget_partition.shutdown()

    def isChecked(self, idx):
        return self.widget_buttons.isChecked(idx)

    def resetTransform(self):
        self.widget_partition.resetTransform()

    def resetPolygons(self):
        self.widget_partition.resetPolygons()


class ButtonsBox(QtWidgets.QWidget):
    def __init__(self, global_widget, parent=None):
        QtWidgets.QWidget.__init__(self, parent)
        self.global_widget = global_widget
        
        layout_all = QHBoxLayout(self)
        layout_left = QVBoxLayout(self)
        layout_right = QVBoxLayout(self)
        self.buttons_robot = []
        for r in range(len(robot_names)):
            b = QPushButton(robot_names[r])
            b.setCheckable(True)
            b.toggle()
            b.setGeometry(QRect(QPoint(100,100),QSize(200,50)))
            color = robot_colors[r].name()
            b.setStyleSheet("background-color:" + color);
            layout_left.addWidget(b)
            #b.clicked.connect(lambda: self.clickCallback(i) )
            self.buttons_robot.append(b)
        self.tools_names = ['reset view', 'reset robots']
        self.buttons_tools = []
        for i in range(len(self.tools_names)):
            b = QPushButton(self.tools_names[i])
            b.setCheckable(False)
            b.toggle()
            b.setGeometry(QRect(QPoint(100,100),QSize(200,50)))
            layout_right.addWidget(b)
            # b.clicked.connect(lambda i: self.clickToolsCallback(i) )
            b.clicked.connect(partial(self.clickToolsCallback, i) )
            self.buttons_tools.append(b)
        layout_all.addLayout(layout_left,1)
        layout_all.addLayout(layout_right,1)
        self.setLayout(layout_all)

    def clickToolsCallback(self, idx):
        #print(idx)
        if idx==0:
            self.global_widget.resetTransform()
        if idx==1:
            self.global_widget.resetPolygons()
            self.global_widget.resetTransform()

    def isChecked(self, idx):
        return self.buttons_robot[idx].isChecked()


class RobotPartition():
    def __init__(self, name, idx):
        # stuff related to polygon 
        self.pen = QtGui.QPen(QtGui.QColor(0,0,0))                      # set lineColor
        self.pen.setWidth(3)                                            # set lineWidth
        self.brush = QtGui.QBrush(robot_colors[idx])        # set fillColor  
        self.idx = idx                       
        self.name = name
        self.initialisePolygon()

    def initialisePolygon(self):
        idx = self.idx
        #self.polygon = self.createPoly(6,10,20+1*idx,20+1*idx,0)  # polygon with n points, radius, angle of the first point
        # overlap = 10
        # angle_width = overlap + 180/3
        # start = -90 - overlap/2 + idx * 180/3
        # end = start + angle_width
        # r = 200
        # self.polygon = self.createPoly(start, end, r)

        y_max = 200
        x_max = 200

        if idx==0:
            polygon = QtGui.QPolygon()
            polygon.append(QtCore.QPoint(0, 0))
            polygon.append(QtCore.QPoint(0, y_max))
            polygon.append(QtCore.QPoint(x_max, y_max))
            polygon.append(QtCore.QPoint(x_max, 0))
            self.polygon = polygon
        elif idx==1:
            polygon = QtGui.QPolygon()
            polygon.append(QtCore.QPoint(0, 0))
            polygon.append(QtCore.QPoint(0, -y_max))
            polygon.append(QtCore.QPoint(x_max, -y_max))
            polygon.append(QtCore.QPoint(x_max, 0))
            self.polygon = polygon
        elif idx>1:
            polygon = QtGui.QPolygon()
            polygon.append(QtCore.QPoint(x_max/2, -y_max/2))
            polygon.append(QtCore.QPoint(x_max, -y_max/2))
            polygon.append(QtCore.QPoint(x_max, y_max/2))
            polygon.append(QtCore.QPoint(x_max/2, y_max/2))
            self.polygon = polygon


    # def createPoly(self, start_angle, end_angle, r):
    #     polygon = QtGui.QPolygon()
    #     polygon.append(QtCore.QPoint(0, 0))
    #     polygon.append(QtCore.QPoint(r*math.cos(math.radians(start_angle)), r*math.sin(math.radians(start_angle))))
    #     polygon.append(QtCore.QPoint(r*math.cos(math.radians(end_angle)), r*math.sin(math.radians(end_angle))))
    #     return polygon

    # def createPoly(self, n, r, x_offset, y_offset, s):
    #     polygon = QtGui.QPolygon() 
    #     w = 360/n                                                       # angle per step
    #     for i in range(n):                                              # add the points of polygon
    #         t = w*i + s
    #         x = r*math.cos(math.radians(t))
    #         y = r*math.sin(math.radians(t))
    #         polygon.append(QtCore.QPoint(x_offset + x, y_offset + y))  

    #     return polygon

    def isSimplePolygon(self):
        list = []
        for i in range(self.polygon.size()):
            p = self.polygon.at(i)
            t = (p.x(),p.y())
            list.append(t)
        polygon_convert = geometry.Polygon(list)
        return polygon_convert.is_valid

class MapWidget(QtWidgets.QWidget):
    def __init__(self, global_widget, parent=None):
        QtWidgets.QWidget.__init__(self, parent)
        self.global_widget = global_widget

        self.partitions = []
        for r in range(len(robot_names)):
            p = RobotPartition(robot_names[r], r)
            self.partitions.append(p)            

        self.size_units = 200
        self.index_clicked = None
        self.robot_clicked = None
        self.click_point_x = None
        self.click_point_y = None
        self.click_point_image_x = None
        self.click_point_image_y = None
        self.moving_polygon = -1
        self.mouseReleaseEvent=self.mouseReleaseEventCallback
        self.mousePressEvent=self.mousePressEventCallback
        self.mouseMoveEvent=self.mouseMoveEventCallback
        self.TOLERANCE = 20

        # ros tranform listener to tranform odometry
        self.listener = tf.TransformListener()

        self.s_inverse = 1
        self.translation = [0, 0]
        self.transform = QtGui.QTransform()
        self.transform_inverse = None 
        self.scroll_wheel_scaling = 1
        self.resetTransform()

        # subscribe to /integrated_to_map from each robot (and do one without prefix just in case
        self.keypose_sub_without_prefix = rospy.Subscriber("/integrated_to_map", Odometry,self.keypose_callback)
        self.keypose_sub = []
        for r in range(len(robot_names)):
            self.keypose_sub.append(rospy.Subscriber("/" + robot_names[r] + "/frame/integrated_to_map", Odometry,self.keypose_callback))
        self.pose_list = QtGui.QPolygonF()

        for p in self.partitions:
        	publisher_name = '{:s}_pub'.format(p.name)
        	setattr(self, publisher_name, rospy.Publisher('/' + p.name + '/coordination_polygon_bst' , PolygonStamped, queue_size=10))

        # setup a timer
        self.timer_publisher = rospy.Timer(rospy.Duration(1), self.publishTimerCallback)

        self.never_drawn = True

    def shutdown(self):
        try:
            self.timer_publisher
        except NameError:
            self.timer_publisher = None
        if self.timer_publisher:
            self.timer_publisher.shutdown()

    def keypose_callback(self, msg):
        
        try:
            # transform from the robot's /<robot>_map frame to /darpa_map frame
            # so that it is displayed in the same frame as the polygons
            
            # create a PoseStamped message
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = msg.pose.pose

            # transform the message
            from_frame = pose_stamped.header.frame_id
            target_frame = '/darpa_map'
            # time syncing issue if you don't wait until tf arrives
            self.listener.waitForTransform(from_frame, target_frame, pose_stamped.header.stamp, rospy.Duration(0.1));
            transformed_pose_stamped = self.listener.transformPose(target_frame, pose_stamped)

            # add tranformed data to list
            self.pose_list.append(QtCore.QPointF(transformed_pose_stamped.pose.position.x, transformed_pose_stamped.pose.position.y))

            rospy.logwarn('transform correct')

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # just ignore this exception (no keypose will be displayed, everything else works)
            print(e)
            rospy.logwarn('CoordinationPlugin: error transforming odometry')
        
    
    def resetTransform(self):

        self.scroll_wheel_scaling = 0.7

        self.s_inverse = 1

        h = self.height()
        w = self.width()
        a = min(h,w)
        s = float(self.scroll_wheel_scaling) * float(a) / float(self.size_units)
        self.s_inverse = s**-1

        self.translation = [w/2, h*0.9]

        self.transform = QtGui.QTransform()
        self.transform_inverse = None 

        self.update()

    def resetPolygons(self):
        for r in range(len(self.partitions)):
            self.partitions[r].initialisePolygon()
        self.update()

    def transformPixelsToMap(self, x, y):        
        if self.transform_inverse != None:
            point = QPoint(x,y)
            transformed_point = self.transform_inverse.map(point)
            xt = transformed_point.x()
            yt = transformed_point.y()
            return [xt, yt]
        else:
            return [x, y]

    def mouseReleaseEventCallback(self, event):
        # clear
        self.index_clicked = None
        self.robot_clicked = None
        self.click_point_x = None
        self.click_point_y = None
        self.click_point_image_x = None
        self.click_point_image_y = None
        self.moving_polygon = -1
        self.update()

    def mousePressEventCallback(self, event):
        # remember where I clicked for 'move' behaviour
        # or, if right click, remove a vertex

        [x, y] = self.transformPixelsToMap(event.x(), event.y()) 

        self.click_point_image_x = event.x()
        self.click_point_image_y = event.y()

        for r in range(len(robot_names)):

                polygon = self.partitions[r].polygon
                self.click_point_x = x
                self.click_point_y = y
                

                if event.button() == Qt.RightButton:

                    # see if the click matches a vertex
                    # remove it
                               
                    index = None
                    for i in range(polygon.size()):
                        if (polygon.point(i).x()-x)**2 + (polygon.point(i).y()-y)**2 <= (self.TOLERANCE*self.s_inverse)**2: #/self.s_inverse
                            index = i
                    if index != None:
                        saved_point = polygon.at(index)
                        self.partitions[r].polygon.remove(index)
                        if not self.partitions[r].isSimplePolygon():
                            # don't let the polygon become complex
                            self.partitions[r].polygon.insert(index, saved_point)
                        self.index_clicked = None
                        self.robot_clicked = None
                        self.update()


    def mouseMoveEventCallback(self, event):
        # move a vertex, or insert vertex on edge, or move the entire polygon
        # code follows that order of precedence when checking for condition

        [x, y] = self.transformPixelsToMap(event.x(), event.y())
        
        not_in_polygon = True

        for r in range(len(robot_names)):
            index = None

            if self.global_widget.isChecked(r) and (self.robot_clicked == r or self.robot_clicked == None):
                polygon = self.partitions[r].polygon

                if self.index_clicked != None and self.moving_polygon == -1:
                    saved_point = polygon.at(self.index_clicked)
                    polygon.setPoint(self.index_clicked, x, y)
                    if not self.partitions[r].isSimplePolygon():
                        # don't let the polygon become complex
                        polygon.setPoint(self.index_clicked, saved_point)
                    self.update()
                    
                    return

                # see if the click matches a vertex
                for i in range(polygon.size()):
                    if (polygon.point(i).x()-x)**2 + (polygon.point(i).y()-y)**2 <= (self.TOLERANCE*self.s_inverse)**2:
                        index = i
                if index != None and self.moving_polygon == -1:
                    polygon.setPoint(index, x, y)
                    self.index_clicked = index
                    self.robot_clicked = r
                    self.update()
                    return


                # see if the click matches an edge
                point = np.array([x, y])
                n = polygon.size()
                for i in range(n-1):
                    line = np.array([[polygon.point(i).x(), polygon.point(i).y()], [polygon.point(i+1).x(), polygon.point(i+1).y()]])
                    dist = point_to_line_dist(point, line)
                    if dist <= self.TOLERANCE*self.s_inverse:
                        index = i
                line = np.array([[polygon.point(n-1).x(), polygon.point(n-1).y()], [polygon.point(0).x(), polygon.point(0).y()]])
                dist = point_to_line_dist(point, line)
                if dist <= self.TOLERANCE*self.s_inverse:
                    index = n-1
                if index != None and self.moving_polygon == -1:
                    polygon.insert(index+1, QtCore.QPoint(x,y))
                    self.update()
                    return


                # see if I'm in the polygon, if so, move it
                if ( polygon.containsPoint(QtCore.QPoint(x,y), Qt.OddEvenFill) and self.moving_polygon == -1 ) or (self.moving_polygon == r):
                    if self.click_point_x != None:
                        polygon.translate(x-self.click_point_x, y-self.click_point_y)
                        self.click_point_x = x
                        self.click_point_y = y
                        self.moving_polygon = r
                        self.update()
                        return
            else:
                polygon = self.partitions[r].polygon
                if polygon.containsPoint(QtCore.QPoint(x,y), Qt.OddEvenFill):
                    not_in_polygon = False


        if not_in_polygon:
            # none of the above happened, and I'm not in an inactive polygon
            # in this case, transform the entire map
            self.translation[0] += event.x()-self.click_point_image_x
            self.translation[1] += event.y()-self.click_point_image_y
            self.click_point_image_x = event.x()
            self.click_point_image_y = event.y()
            self.update()
            return

    def wheelEvent(self,event):
        qp = event.angleDelta()/120 
        scaling = 1.15**(qp.y())
        self.scroll_wheel_scaling *= scaling

        # scale about mouse point
        self.translation[0] = scaling*(self.translation[0] - event.x()) + event.x()
        self.translation[1] = scaling*(self.translation[1] - event.y()) + event.y()
        self.update()

    def getPainter(self):
        
        painter = QtGui.QPainter(self)
        #transform_ = QtGui.QTransform()
        #transform_.translate(250,250)
        #transform_.rotate(-90)
        #transform_.translate(-250,-250)
        #transform_.scale(1,-1)
        #painter.setWorldTransform(transform_)

        # set translation
        painter.translate(self.translation[0],self.translation[1])
        painter.rotate(-90)

        #painter.translate(250,250)
        #transform_ = QtGui.QTransform()
        #transform_.rotate
        #painter.setTransform(transform_)
        
        #painter.translate(-250,-250)
        #painter.translate(-self.translation[0], -self.translation[1])
        #painter.rotate(-90)
        #transform_ = QtGui.QTransform()
    	#transform_.translate(0, 0)
    	#transform_.rotate(90)
    	#transform_.scale(1, -1.0)
    	#painter.setTransform(transform_)
        # set scale
        h = self.height()
        w = self.width()
        a = min(h,w)
        s = float(self.scroll_wheel_scaling) * float(a) / float(self.size_units)
        self.s_inverse = s**-1
        painter.scale(s, -s)
        return painter

    def paintEvent(self, event):
        painter = self.getPainter()

        self.transform = painter.combinedTransform()
        [self.transform_inverse, inverse_exists] = self.transform.inverted()
        if not inverse_exists:
            print('WARNING inverse doesnt exist')

        # draw stuff     
        for p in self.partitions:
            p.pen.setWidthF(3.0*self.s_inverse)
            painter.setPen(p.pen)
            painter.setBrush(p.brush)  
            painter.drawPolygon(p.polygon)

        # highlight the last few received points
        num_points_to_highlight = 8
        start_idx = len(self.pose_list) - 1 - num_points_to_highlight
        if start_idx < 0:
            start_idx = 0
        pen_ = QtGui.QPen(QtGui.QColor(255,200,0))
        pen_.setWidthF(20.0*self.s_inverse)
        painter.setPen(pen_)
        painter.drawPoints(self.pose_list[start_idx:])

        # draw the points
    	pen_ = QtGui.QPen(QtGui.QColor(0,0,0))
    	pen_.setWidthF(5.0*self.s_inverse)
    	painter.setPen(pen_)
    	painter.drawPoints(self.pose_list)

       	# draw origin frame
        origin_size = 20
       	pen_ = QtGui.QPen(QtGui.QColor(255,0,0)) 
       	pen_.setWidthF(5.0*self.s_inverse)
       	painter.setPen(pen_)
       	painter.drawLine(0,0,origin_size,0)
       	pen_.setColor(QtGui.QColor(0,255,0))
       	painter.setPen(pen_)
       	painter.drawLine(0,0,0,origin_size)

        if self.never_drawn:
            self.never_drawn = False
            self.resetTransform()
            self.update()


    def publishTimerCallback(self, timer_event):

       	# publish the polygons.       	
       	for p in self.partitions:
       		publisher_name = '{:s}_pub'.format(p.name)
       		poly_msg = PolygonStamped()
       		poly_msg.header.frame_id = '/darpa_map'
       		for i in p.polygon:
       			point = Point32()
       			point.x = i.x()
       			point.y = i.y()
       			point.z = 0.0
       			poly_msg.polygon.points.append(point)
       		getattr(self, publisher_name).publish(poly_msg)

        # redraw
        self.update()


# app = QtWidgets.QApplication(sys.argv) 

# plugin = CoordinationPlugin()
# plugin.show()

# signal.signal(signal.SIGINT, signal.SIG_DFL)

# sys.exit(app.exec_())
