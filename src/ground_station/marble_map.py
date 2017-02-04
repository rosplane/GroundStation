from PyKDE4.marble import *
import map_info_parser

class MarbleMap(Marble.MarbleWidget):
    def __init__(self, parent=None): # Parent line VERY important
        super(MarbleMap, self).__init__() # MarbleWidget constructor
        self.setMapThemeId("earth/openstreetmap/openstreetmap.dgml")
        self.setProjection(Marble.Mercator)
        self.setShowOverviewMap(False)

        self._map_coords = map_info_parser.get_gps_dict()
        def_latlonzoom = self._map_coords[map_info_parser.get_default()]
        self._home_pt = Marble.GeoDataCoordinates(def_latlonzoom[1], def_latlonzoom[0], 0.0, Marble.GeoDataCoordinates.Degree) # +
        self.centerOn(self._home_pt)
        self.zoomView(def_latlonzoom[2])

    def change_home(self, map_name):
        latlonzoom = self._map_coords[map_name]
        self._home_pt = Marble.GeoDataCoordinates(latlonzoom[1], latlonzoom[0], 0.0, Marble.GeoDataCoordinates.Degree)
        self.centerOn(self._home_pt)
        self.zoomView(latlonzoom[2])
