from PyKDE4.marble import *

class MarbleMap(Marble.MarbleWidget):
    def __init__(self, parent=None): # Parent line VERY important
        super(MarbleMap, self).__init__() # MarbleWidget constructor
        self.setMapThemeId("earth/openstreetmap/openstreetmap.dgml")
        self.setProjection(Marble.Mercator)
        self.setShowOverviewMap(False)
        # default home is on naval base
        self._home_pt = Marble.GeoDataCoordinates(-76.429454, 38.146191, 0.0, Marble.GeoDataCoordinates.Degree)
        self.centerOn(self._home_pt)
        self.zoomView(2850)
    def change_home(self, latlon):
        # for a future home toggler (example: changing between BYU and naval base)
        self._home_pt = Marble.GeoDataCoordinates(latlon[1], latlon[0], 0.0, Marble.GeoDataCoordinates.Degree)
        self.centerOn(self._home_pt)
        self.zoomView(2850)
