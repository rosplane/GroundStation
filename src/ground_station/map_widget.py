from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from PyKDE4.marble import *

from .manage_kml import ManageKML
from .marble_map import MarbleMap
from .wp_window import WpWindow
from .cm_window import CmWindow
import map_info_parser
import os

from PyQt4.QtGui import *
from PyQt4.QtCore import *

PWD = os.path.dirname(os.path.abspath(__file__))

class MapWindow(QWidget):
    def __init__(self, uifname = 'map_widget.ui'):
        super(MapWindow, self).__init__()
        button_icon_file = os.path.join(PWD, 'resources', 'airplane.png')
        ui_file = os.path.join(PWD, 'resources', uifname)
        loadUi(ui_file, self, {'MarbleMap' : MarbleMap})
        # there is now a self._marble_map, and there can only be one
        #self._marble_map.inputHandler.setMouseButtonPopupEnabled(Qt.RightButton, False)
        self.setObjectName(uifname)
        self.interval = 100     # in milliseconds, period of regular update
        self.timer = QTimer(self)
        self.timer.setInterval(self.interval)
        self.connect(self.timer, SIGNAL('timeout()'), self._marble_map.update)

        map_coords = map_info_parser.get_gps_dict()
        self._home_opts.clear()
        self._home_opts.addItems(list(map_coords))
        self._home_opts.setCurrentIndex(list(map_coords).index(map_info_parser.get_default()))
        self._home_opts.currentIndexChanged[str].connect(self._update_home)

        self.init_manage_kml()
        self.init_wp_window()
        self.init_cm_window()
        self.timer.start()

    def init_manage_kml(self):
        self.manageKML = ManageKML(self._marble_map)
        self._manage_KML.clicked.connect(self.manageKML.display_manage_KML_modal)
        self.manageKML.add_default_KML_files()

    def init_wp_window(self):
        self.wpWindow = WpWindow(self._marble_map)
        self._send_WP.clicked.connect(self.open_wp_window)

    def init_cm_window(self):
        self.cmWindow = CmWindow(self._marble_map)
        self._special_commands.clicked.connect(self.open_cm_window)

    def open_wp_window(self):
        #self._marble_map.setInputEnabled(False)
        #self._marble_map._mouse_attentive = True
        self.wpWindow.show()

    def open_cm_window(self):
        self._marble_map.setInputEnabled(False)
        self._marble_map._mouse_attentive = True
        self.cmWindow.show()

    def _update_home(self):
        self._marble_map.change_home(self._home_opts.currentText())

    def close(self):
        self.timer.stop()
        super(MapWindow, self).close()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
