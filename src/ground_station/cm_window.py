from python_qt_binding import loadUi
from PyQt4.Qt import *
from PyQt4 import QtGui

try:
    from PyQt4.QtCore import QString
except ImportError:
    QString = type("")

import os
import map_info_parser
import rospy
from fcu_common.msg import FW_Waypoint
from std_msgs.msg import Bool

PWD = os.path.dirname(os.path.abspath(__file__))
RTH_ALT = 10 # "return to home" command will have the plane fly 10 m above home pt

class Override_WP_Publisher(): # for loitering, RTH
    def __init__(self):
        self.pub = rospy.Publisher('/waypoint_path', FW_Waypoint, queue_size=50)

    def publish_wp_to_plane(self, wp):
        wp_obj = FW_Waypoint()
        wp_obj.w[0] = wp[0]
        wp_obj.w[1] = wp[1]
        wp_obj.w[2] = wp[2]
        wp_obj.chi_d = 0.0 # course for this waypoint
        wp_obj.chi_valid = False # determines if dubins is used
                                 # (see ros_plane -> path_manager_example.cpp)
        wp_obj.Va_d = 30.0 # m/s
        wp_obj.set_current = True # sets to be executed now
        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # MAKE THE NEW FIELD, DEFAULT = False ++++++++++++++++++++++++++++++++++++++++++++
        #wp_obj.override = True

        self.pub.publish(wp_obj)

class CmWindow(QWidget):

    #zs_pub = +++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #to_pub = +++++++++++++++++++++++++++++++++++++++++++++++++++++++
    drop_pub = rospy.Publisher('bomb_drop', Bool, queue_size=5)

    def __init__(self, marble, uifname = 'cm_window.ui'):
        super(CmWindow, self).__init__()
        self.marble = marble
        ui_file = os.path.join(PWD, 'resources', uifname)
        loadUi(ui_file, self)
        self.setObjectName(uifname)

        self.zero_sensor_button.clicked.connect(self.zero_sensor_command)
        self.takeoff_button.clicked.connect(self.takeoff_command)
        self.loiter_button.clicked.connect(self.loiter_command)
        self.rth_button.clicked.connect(self.rth_command)
        self.land_button.clicked.connect(self.land_command)
        self.drop_button.clicked.connect(self.drop_command)
        self.marble.WPH.wp_clicked.connect(self.clicked_waypoint)

        # updating not needed, unless zero_sensors somehow comes to need it
        #self._home_map = self.marble._home_map
        # self.marble.latlon gives home location

        self.OWPP = Override_WP_Publisher()

    def zero_sensor_command(self):
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        print('zero sensor functionality pending')

    def takeoff_command(self):
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        print ('takeoff functionality pending')

    def loiter_command(self):
        if self.marble.GIS.received_msg:
            try:
                lat = float(str(self.loiter_lat_field.toPlainText()))
                lon = float(str(self.loiter_lon_field.toPlainText()))
                alt = float(str(self.loiter_alt_field.toPlainText()))
                meter_data = self.marble.GIS.GB.gps_to_ned(lat, lon, alt/3.281)
                self.OWPP.publish_wp_to_plane(meter_data)
            except ValueError:
                print('Incorrectly formatted fields. Must all be numbers.')

    def rth_command(self):
        if self.marble.GIS.received_msg:
            lat = self.marble.latlon[0]
            lon = self.marble.latlon[1]
            meter_data = self.marble.GIS.GB.gps_to_ned(lat, lon, RTH_ALT)
            self.OWPP.publish_wp_to_plane(meter_data)

    def land_command(self):
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # maybe just make a OWP at altitude = 0, leading the plane on? +++++++++++
        print('landing functionality pending')

    def drop_command(self):
        self.drop_pub.publish(True)

    def clicked_waypoint(self, lat, lon):
        self.loiter_lat_field.setText(QString(str(lat)))
        self.loiter_lon_field.setText(QString(str(lon)))

    def closeEvent(self, QCloseEvent):
        self.marble.setInputEnabled(True)
        self.marble._mouse_attentive = False
