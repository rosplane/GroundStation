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
from ros_plane.msg import Waypoint

PWD = os.path.dirname(os.path.abspath(__file__))

class WP_Publisher():
    def __init__(self):
        self.pub = rospy.Publisher('/waypoint_path', Waypoint, queue_size=50)

    def publish_wp_to_plane(self, wp):
        wp_obj = Waypoint()
        wp_obj.w[0] = wp[0]
        wp_obj.w[1] = wp[1]
        wp_obj.w[2] = wp[2]
        wp_obj.chi_d = 0.0 # course for this waypoint
        wp_obj.chi_valid = False # determines if dubins is used
                                 # (see ros_plane -> path_manager_example.cpp)
        wp_obj.Va_d = 30.0 # m/s
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
        self.pushButton.clicked.connect(self.add_waypoint)
        self.pushButton_2.clicked.connect(self.remove_waypoint)
        self.send_to_plane_button.clicked.connect(self.transfer_waypoint_data)
        self.mode_comboBox.clear()
        self.mode_comboBox.addItem(QString('Empty Mode'))
        self.mode_comboBox.addItem(QString('Main Mode'))
        self.mode_comboBox.addItem(QString('Search Mode'))
        self.mode_comboBox.currentIndexChanged.connect(self.load_wp_mode)

        # For signal handling
        self.marble.WPH.wp_clicked.connect(self.clicked_waypoint)
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
        #self.transfer_waypoint_data() # update plane's waypoints ============================================

    def remove_waypoint(self): # no home or mode change
        pos = self.listWidget.currentRow()
        if pos > -1:
            del self.waypoints[pos] # update self.waypoints
            self.marble.WPH.emit_removed(pos) # update map waypoints
            self.update_lists() # update wp_window contents
            #self.transfer_waypoint_data() # update plane's waypoints ========================================

    def add_waypoint(self): # no home or mode change
        # Check PARAMS and emit signal
        try:
            lat = float(str(self.textEdit.toPlainText()))
            lon = float(str(self.textEdit_2.toPlainText()))
            alt = float(str(self.textEdit_3.toPlainText()))
            pos = int(str(self.comboBox.currentText()))
            self.waypoints.insert(pos, (lat, lon, alt)) # update self.waypoints
            self.update_lists() # update wp_window contents
            self.marble.WPH.emit_inserted(lat, lon, alt, pos) # update map waypoints
            #self.transfer_waypoint_data() # update plane's waypoints ============================================
        except ValueError:
            print('Incorrectly formatted fields. Must all be numbers.')

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
        self.comboBox.clear()
        i = 0
        for waypoint in self.waypoints:
            self.listWidget.addItem(QString(str(i)+': '+str(waypoint)))
            self.comboBox.addItem(QString(str(i)))
            i += 1
        self.comboBox.addItem(QString(str(i)))

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
                wp_file.write('%f %f %f\n' % (wp[0], wp[1], wp[2]))

    def transfer_waypoint_data(self): # needs new self.waypoints
        if self.marble.GIS.received_msg and not self.marble.wp_state == 'None':
            #self.WPP.publish_wp_to_plane([-9999,-9999,-9999]) # "Reset" wp
            for wp in self.waypoints:
                meter_data = self.marble.GIS.GB.gps_to_ned(wp[0], wp[1], wp[2]/3.28084)
                self.WPP.publish_wp_to_plane(meter_data)

    # MISCELLANEOUS

    def clicked_waypoint(self, lat, lon):
        self.textEdit.setText(QString(str(lat)))
        self.textEdit_2.setText(QString(str(lon)))

    def closeEvent(self, QCloseEvent):
        self.save_waypoints()
        self.marble.setInputEnabled(True)
        self.marble._mouse_attentive = False
