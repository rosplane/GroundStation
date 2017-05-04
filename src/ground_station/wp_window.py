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

PWD = os.path.dirname(os.path.abspath(__file__))

class WP_Publisher():
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
        wp_obj.set_current = False # sets to be executed now
        self.pub.publish(wp_obj)

class WpWindow(QWidget):
    def __init__(self, _marble_map, uifname = 'wp_window.ui'):
        super(WpWindow, self).__init__()
        self._marble_map = _marble_map
        ui_file = os.path.join(PWD, 'resources', uifname)
        loadUi(ui_file, self)
        self.setObjectName(uifname)

        self._home_map = self._marble_map._home_map
        self.waypoints = map_info_parser.get_waypoints(self._home_map)

        i = 0
        for waypoint in self.waypoints:
            self.listWidget.addItem(QString(str(i)+': '+str(waypoint)))
            self.comboBox.addItem(QString(str(i)))
            i += 1
        self.comboBox.addItem(QString(str(i)))

        self.pushButton.clicked.connect(self.add_waypoint)
        self.pushButton_2.clicked.connect(self.remove_waypoint)
        self.pushButton_3.clicked.connect(self.load_wp_file)
        # For signal handling
        self._marble_map.WPH.wp_clicked.connect(self.clicked_waypoint)
        self._marble_map.WPH.home_changed.connect(self.change_home)

        self.WPP = WP_Publisher()

    def load_wp_file(self):
        filename = str(QtGui.QFileDialog.getOpenFileName(self, 'Open File', PWD)[0])
        if not filename.strip() == '':
            try:
                new_wps = []
                with open(filename.strip(), 'r') as wp_file:
                    for line in wp_file:
                        wp_t = line.split()
                        lat = float(wp_t[0])
                        lon = float(wp_t[1])
                        alt = float(wp_t[2])
                        new_wps.append((lat, lon, alt))
                for i in range(len(self.waypoints)):
                    self._marble_map.WPH.emit_removed(0)
                self.waypoints = new_wps
                self.update_lists()
                for i, wp in enumerate(self.waypoints):
                    self._marble_map.WPH.emit_inserted(wp[0], wp[1], wp[2], i)
                self.transfer_waypoint_data()
            except:
                print('Invalid waypoint file selected. Ensure that format is correct.')

    def update_lists(self):
        self.listWidget.clear()
        self.comboBox.clear()
        i = 0
        for waypoint in self.waypoints:
            self.listWidget.addItem(QString(str(i)+': '+str(waypoint)))
            self.comboBox.addItem(QString(str(i)))
            i += 1
        self.comboBox.addItem(QString(str(i)))

    def save_waypoints(self):
        wp_file_path = os.path.join(PWD, 'resources', 'wp_data', '%s_wp_data.txt' % self._home_map)
        with open(wp_file_path, 'w') as wp_file:
            for wp in self.waypoints:
                wp_file.write('%f %f %f\n' % (wp[0], wp[1], wp[2]))

    def transfer_waypoint_data(self):
        wp_file_path = os.path.join(PWD, 'resources', 'wp_data', '%s_wp_data.txt' % self._home_map)
        self.save_waypoints()
        with open(wp_file_path, 'r') as wp_file:
            self.WPP.publish_wp_to_plane([-9999,-9999,-9999]) # "Reset" wp
            for line in wp_file:
                wp_t = line.split()
                lat = float(wp_t[0])
                lon = float(wp_t[1])
                alt = float(wp_t[2])
                meter_data = self._marble_map.GB.gps_to_ned(lat, lon, alt/3.281)
                self.WPP.publish_wp_to_plane(meter_data)

    def add_waypoint(self):
        # Check PARAMS and emit signal
        try:
            lat = float(str(self.textEdit.toPlainText()))
            lon = float(str(self.textEdit_2.toPlainText()))
            alt = float(str(self.textEdit_3.toPlainText()))
            pos = int(str(self.comboBox.currentText()))
            self.waypoints.insert(pos, (lat, lon, alt))
            self.update_lists()
            self._marble_map.WPH.emit_inserted(lat, lon, alt, pos)
            self.transfer_waypoint_data()
        except ValueError:
            print('Incorrectly formatted fields. Must all be numbers.')

    def remove_waypoint(self):
        pos = self.listWidget.currentRow()
        if pos > -1:
            del self.waypoints[pos]
            self.update_lists()
            self._marble_map.WPH.emit_removed(pos)
            self.transfer_waypoint_data()

    def clicked_waypoint(self, lat, lon):
        self.textEdit.setText(QString(str(lat)))
        self.textEdit_2.setText(QString(str(lon)))

    def change_home(self, new_home):
        self.save_waypoints()
        self.waypoints = map_info_parser.get_waypoints(new_home)
        self.update_lists()
        self._home_map = new_home

    def closeEvent(self, QCloseEvent):
        self.save_waypoints()
        self._marble_map.setInputEnabled(True)
        self._marble_map._mouse_attentive = False
