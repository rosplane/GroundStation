#from python_qt_binding.QtWidgets import QWidget
from PyKDE4.marble import *

# Should probably move this functionality to another file and make this class
# a more intermediary class with toggler (and .kml loader?)
class MapWindow(Marble.MarbleWidget):
    def __init__(self, latlon):
        super(MapWindow, self).__init__()
        self.setMapThemeId("earth/openstreetmap/openstreetmap.dgml")
        self.setProjection(Marble.Mercator)
        self.setShowOverviewMap(False)
        # default home is on naval base
        self._home = Marble.GeoDataCoordinates(latlon[1], latlon[0], 0.0, Marble.GeoDataCoordinates.Degree)
        self.centerOn(self._home)
        self.zoomView(2850)
    def change_home(self, latlon):
        # for a future home toggler (example: changing between BYU and naval base)
        self._home = Marble.GeoDataCoordinates(latlon[1], latlon[0], 0.0, Marble.GeoDataCoordinates.Degree)
        self.centerOn(self._home)
        self.zoomView(2400)
    def close(self):
        super(MapWindow, self).close()
    def save_settings(self, plugin_settings, instance_settings):
        pass
    def restore_settings(self, plugin_settings, instance_settings):
        pass
