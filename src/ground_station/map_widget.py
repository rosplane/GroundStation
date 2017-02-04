from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from PyKDE4.marble import *

from .marble_map import MarbleMap
import map_info_parser
import os

PWD = os.path.dirname(os.path.abspath(__file__))

class MapWindow(QWidget):
    def __init__(self, uifname = 'map_widget.ui'):
        super(MapWindow, self).__init__()
        ui_file = os.path.join(PWD, 'resources', uifname)
        loadUi(ui_file, self, {'MarbleMap' : MarbleMap})
        # there is now a self._marble_map, and there can only be one
        self.setObjectName(uifname)

        map_coords = map_info_parser.get_gps_dict()
        self._home_opts.clear()
        self._home_opts.addItems(list(map_coords))
        self._home_opts.setCurrentIndex(list(map_coords).index(map_info_parser.get_default()))
        self._home_opts.currentIndexChanged[str].connect(self._update_home)

    def _update_home(self):
        self._marble_map.change_home(self._home_opts.currentText())

    def close(self):
        super(MapWindow, self).close()
    def save_settings(self, plugin_settings, instance_settings):
        pass
    def restore_settings(self, plugin_settings, instance_settings):
        pass
