from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from PyKDE4.marble import *

from .marble_map import MarbleMap
import os

PWD = os.path.dirname(os.path.abspath(__file__))

map_coords = {'Competition':[38.146191, -76.429454],'BYU':[40.2518,-111.6493]}
default_location = 'Competition' # ** Change if needed <^ CHANGE THIS MESS TO A LITTLE XML DB

class MapWindow(QWidget):
    def __init__(self, uifname = 'map_widget.ui'):
        super(MapWindow, self).__init__()
        ui_file = os.path.join(PWD, 'resources', uifname)
        loadUi(ui_file, self, {'MarbleMap' : MarbleMap})
        # there is now a self._marble_map, and there can only be one
        self.setObjectName(uifname)

        self._home_opts.clear()
        self._home_opts.addItems(list(map_coords))

        self._home_opts.setCurrentIndex(list(map_coords).index(default_location))
        self._home_opts.currentIndexChanged[str].connect(self._update_home)

    def _update_home(self):
        self._marble_map.change_home(map_coords[self._home_opts.currentText()])
    def close(self):
        super(MapWindow, self).close()
    def save_settings(self, plugin_settings, instance_settings):
        pass
    def restore_settings(self, plugin_settings, instance_settings):
        pass
