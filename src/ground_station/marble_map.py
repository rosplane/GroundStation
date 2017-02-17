from PyKDE4.marble import *
import map_info_parser
#import rospy
#from fcu_common.msg import FW_Waypoint
#import numpy as np

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
        self.setZoom(def_latlonzoom[2])

        #rospy.Subscriber("/waypoint_path", FW_Waypoint, self.callback_waypoints)
        # waypoint buffers
        #self.wp_pn_data = np.array([])
        #self.wp_pe_data = np.array([])
        #self._curr_wp = self._home_pt

    def change_home(self, map_name):
        latlonzoom = self._map_coords[map_name]
        self._home_pt = Marble.GeoDataCoordinates(latlonzoom[1], latlonzoom[0], 0.0, Marble.GeoDataCoordinates.Degree)
        self.centerOn(self._home_pt)
        self.setZoom(latlonzoom[2])

    def get_home(self):
        return self._home_pt

    #def callback_waypoints(self, FW_Waypoint):
    #    self.wp_pn_data = np.append(self.wp_pn_data, FW_Waypoint.w[0])
    #    self.wp_pe_data = np.append(self.wp_pe_data, FW_Waypoint.w[1])
        # shaky conversion going on here ++++++++++++++++++++++++++++++
    #    lat = self._home_pt.latitude(Marble.GeoDataCoordinates.Degree) + self.wp_pe_data[-1]/111111.0;
    #    lon = self._home_pt.longitude(Marble.GeoDataCoordinates.Degree) + self.wp_pn_data[-1]/111111.0;
    #    self._curr_wp = Marble.GeoDataCoordinates(lon, lat, 0.0, Marble.GeoDataCoordinates.Degree)
    #    self.painter.setPen(Qt.blue)
    #    self.painter.drawEllipse(self._curr_wp, 0.2, 0.2)
        #self.customPaint() nope
    #    print(len(self.wp_pn_data))#-----------------------------------------------
