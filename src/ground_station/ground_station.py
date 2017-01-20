# Standard library items
import rospy
import tf
from tf.transformations import quaternion_from_euler

import numpy
import random
from math import sqrt, atan, pi, degrees

# Ros Messages
from rqt_py_common.topic_helpers import get_field_type # for identifying acceptable topics
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PolygonStamped, PointStamped, PoseWithCovarianceStamped, PoseStamped

# QT Library items
from python_qt_binding.QtCore import Signal, Slot, QPointF, qWarning, Qt
from python_qt_binding.QtGui import QPixmap, QImage, QPainterPath, QPen, QPolygonF, QColor, qRgb
from python_qt_binding.QtWidgets import QWidget, QGraphicsView, QGraphicsScene, QBoxLayout, QVBoxLayout, QHBoxLayout, QPushButton

# Custom Widgets
from .map_widget import MapWindow
from .topic_viewer_widget import TopicViewer
from .control_widget import ControlWindow

def accepted_topic(topic): # checks for valid rostopics to filter through
    msg_types = [OccupancyGrid, Path, PolygonStamped, PointStamped] # imported above
    msg_type, array = get_field_type(topic)

    if not array and msg_type in msg_types:
        return True
    else:
        return False

class GroundStationWidget(QWidget):

    def __init__(self):
        super(GroundStationWidget, self).__init__()

        # The layout of the ground station window
        self._principle_layout = QBoxLayout(0) # main layout is horizontal (0)
        self._map_layout = QVBoxLayout()
        self._principle_layout.addLayout(self._map_layout)
        self._control_layout = QVBoxLayout()
        self._principle_layout.addLayout(self._control_layout)

        self.setAcceptDrops(False) # Dragging and Dropping not permitted
        self.setWindowTitle('ROS_PLANE Ground Station')

        #self.paths = paths#------------------
        #self.polygons = polygons#------------
        #self.map = map_topic#----------------
        #self._tf = tf.TransformListener()#---

        #=============================
        self._mw = MapWindow()
        self._map_layout.addWidget(self._mw)
        self._tv = TopicViewer()
        self._control_layout.addWidget(self._tv, 2) # ratio of these numbers determines window proportions
        self._cw = ControlWindow()
        self._control_layout.addWidget(self._cw, 1)
        #=============================

        #self._set_pose = QPushButton('UB 1')
        #self._set_pose.clicked.connect(self._nav_view.pose_mode) # <<<<<<<<<< Button interface to class
        #self._set_goal = QPushButton('Useless Button 2')
        #self._set_goal.clicked.connect(self._nav_view.goal_mode) # <<<<<<<<<<

        #self._button_layout.addWidget(self._set_pose)
        #self._button_layout.addWidget(self._set_goal)

        #self._layout.addLayout(self._button_layout)

        self.setLayout(self._principle_layout)

    def save_settings(self, plugin_settings, instance_settings):
        #self._nav_view.save_settings(plugin_settings, instance_settings)
        print('fake save') # < prints to terminal

    def restore_settings(self, plugin_settings, instance_settings):
        #self._nav_view.restore_settings(plugin_settings, instance_settings)
        print('fake restore')
