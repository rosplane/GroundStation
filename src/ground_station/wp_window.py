from python_qt_binding import loadUi
from PyQt4.Qt import *
#from PyQt4 import QtGui

try:
    from PyQt4.QtCore import QString
except ImportError:
    QString = type("")

import os, rospy, map_info_parser
from ros_plane.msg import Waypoint

PWD = os.path.dirname(os.path.abspath(__file__))

class WP_Publisher():
    def __init__(self):
        self.pub = rospy.Publisher('/waypoint_path', Waypoint, queue_size=50)

    def publish_wp_to_plane(self, wp, chi, flag, land):
        wp_obj = Waypoint()
        wp_obj.w[0] = wp[0]
        wp_obj.w[1] = wp[1]
        wp_obj.w[2] = wp[2]
        wp_obj.chi_d = chi # course for this waypoint
        wp_obj.chi_valid = True # determines if dubins is used
                                 # (see ros_plane -> path_manager_example.cpp)
        wp_obj.reset = flag # True for the first waypoint sent
        wp_obj.land = land  # True if in landing mode
        wp_obj.Va_d = 15.0 # m/s
        wp_obj.set_current = False # sets to be executed now
        self.pub.publish(wp_obj)

class WpWindow(QWidget):
    def __init__(self, marble, uifname = 'wp_window.ui'):
        super(WpWindow, self).__init__()
        self.marble = marble
        ui_file = os.path.join(PWD, 'resources', uifname)
        loadUi(ui_file, self)
        self.setObjectName(uifname)

        # Set up waypoints and waypoint files
        self._home_map = self.marble._home_map
        self.WPP = WP_Publisher()
        self.load_wp_from_file()
        self.update_lists()
        self.set_title()

        # Set up event triggers
        self.send_to_plane_button.clicked.connect(self.transfer_waypoint_data)
        self.mode_comboBox.clear()
        self.mode_comboBox.addItem(QString('Empty Mode'))
        self.mode_comboBox.addItem(QString('Main Mode'))
        self.mode_comboBox.addItem(QString('Search Mode'))
        self.mode_comboBox.currentIndexChanged.connect(self.load_wp_mode)

        # For signal handling
        self.marble.WPH.home_changed.connect(self.change_home)

    # CHANGE TRIGGERS

    def load_wp_mode(self):
        self.save_waypoints()
        if str(self.mode_comboBox.currentText()) == 'Main Mode':
            self.marble.wp_state = 'MainWP'
        elif str(self.mode_comboBox.currentText()) == 'Search Mode':
            self.marble.wp_state = 'SearchWP'
        else:
            self.marble.wp_state = 'None'
        self.full_update()

    def change_home(self, new_home):
        self.save_waypoints()
        self._home_map = new_home
        self.full_update()

    # UPDATE HANDLERS

    def full_update(self):
        for i in range(len(self.waypoints)): # clear map waypoints
            self.marble.WPH.emit_removed(0)
        self.load_wp_from_file() # update self.waypoints
        self.update_lists() # update wp_window contents
        self.set_title()
        for i, wp in enumerate(self.waypoints): # update map waypoints
            self.marble.WPH.emit_inserted(wp[0], wp[1], wp[2], i)

    # UPDATE FUNCTIONS

    def set_title(self): # needs new home map and wp_state
        if self.marble.wp_state == 'MainWP':
            self.waypoint_label.setText(QString('%s Main Waypoints' % self._home_map))
        elif self.marble.wp_state == 'SearchWP':
            self.waypoint_label.setText(QString('%s Search Waypoints' % self._home_map))
        else:
            self.waypoint_label.setText(QString('Choose a mode for %s' % self._home_map))

    def load_wp_from_file(self): # needs new home map and wp_state
        if self.marble.wp_state == 'MainWP':
            self.waypoints = map_info_parser.get_main_waypoints(self._home_map)
        elif self.marble.wp_state == 'SearchWP':
            self.waypoints = map_info_parser.get_search_waypoints(self._home_map)
        else:
            self.waypoints = []

    def update_lists(self): # needs new self.waypoints
        self.listWidget.clear()
        i = 0
        for waypoint in self.waypoints:
            self.listWidget.addItem(QString(str(i)+': '+str(waypoint)))
            i += 1

    def save_waypoints(self): # needs OLD home map and wp_state
        folder_file_name = ''
        if self.marble.wp_state == 'MainWP':
            folder_file_name = 'main_wps/%s_main_wps.txt' % self._home_map
        elif self.marble.wp_state == 'SearchWP':
            folder_file_name = 'search_wps/%s_search_wps.txt' % self._home_map
        else: # shouldn't be saving anything
            return

        wp_file_path = os.path.join(PWD, 'resources', 'wp_data', folder_file_name)
        with open(wp_file_path, 'w') as wp_file:
            for wp in self.waypoints:
                wp_file.write('%f %f %f %f\n' % (wp[0], wp[1], wp[2], wp[3]))

    def transfer_waypoint_data(self): # needs new self.waypoints
        if self.marble.GIS.received_msg and not self.marble.wp_state == 'None':
            wp = self.waypoints[0]
            meter_data = self.marble.GIS.GB.gps_to_ned(wp[0],wp[1], wp[2]/3.28084 - 22.0)
            self.WPP.publish_wp_to_plane(meter_data, wp[3], True, False)
            for wp in self.waypoints[1:]:
                meter_data = self.marble.GIS.GB.gps_to_ned(wp[0], wp[1], wp[2]/3.28084 - 22.0)
                self.WPP.publish_wp_to_plane(meter_data, wp[3], False, False)

    # MISCELLANEOUS

    def closeEvent(self, QCloseEvent):
        self.save_waypoints()
        self.marble.setInputEnabled(True)
        self.marble._mouse_attentive = False
