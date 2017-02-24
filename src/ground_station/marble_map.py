from PyKDE4.marble import *
from PyQt4.QtCore import *
from PyQt4.QtGui import *

import map_info_parser
import rospy
from fcu_common.msg import FW_Waypoint

'''
For changing color of current waypoint to green:
Use the QTimer (as shown in the layer tutorial) to call the marble.update method
at every set interval [[in fact, do it like map_widget's Plane() class, but in this
MarbleMap() class]],
and have PaintLayer have an internal state-subscriber-like class whose data member
of current waypoint is always updating.
'''
class WaypointSubscriber():
    def __init__(self):
        # subscribing to fcu_common/GPS to get plane latitude and longitude
        self.lat = 0.0 # in degrees
        self.lon = 0.0
        #rospy.Subscriber("/waypoint_path", FW_Waypoint, self.callback_waypoints)

    def callback(self, GPS):
        self.lat = GPS.latitude
        self.lon = -1.0*GPS.longitude

# Class for allowing the widget to paint to the marble map
class PaintLayer(Marble.LayerInterface, QObject):
    def __init__(self, marble):
        QObject.__init__(self)
        self.marble = marble
        self._home_map = self.marble._home_map
        self.waypoints = map_info_parser.get_waypoints(self._home_map)
        # subscriber class +++++++++++++++++++++++++++++++++++

    def renderPosition(self): # ??
        return ['SURFACE']

    def render(self, painter, viewPort, renderPos, layer):
        painter.m_index = 0 # should provide a surface paint
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setPen(QPen(QBrush(Qt.red), 4.5, Qt.SolidLine, Qt.RoundCap))
        # Draw waypoints according to latlong degrees for current map
        self._home_map = self.marble._home_map
        self.waypoints = map_info_parser.get_waypoints(self._home_map)

        for waypoint in self.waypoints:
            location = Marble.GeoDataCoordinates(waypoint[1], waypoint[0], 0.0, Marble.GeoDataCoordinates.Degree)
            painter.drawEllipse(location, 5, 5)

        # Render current target waypoint ++++++++++++++++++++++++++++++

        '''
        for waypoint in self.waypoints_byu:
            location = Marble.GeoDataCoordinates(waypoint[1], waypoint[0], 0.0, Marble.GeoDataCoordinates.Degree)
            painter.drawEllipse(location, 5, 5)
        '''
        return True

class MarbleMap(Marble.MarbleWidget):
    def __init__(self, parent=None): # Parent line VERY important
        super(MarbleMap, self).__init__() # MarbleWidget constructor
        # MAKE IT CHECK FOR GOOGLE MAPS FILES +++++++++++++++++++++++++++++++++
        #self.setMapThemeId("earth/openstreetmap/openstreetmap.dgml") # street view
        self.setMapThemeId("earth/google-maps-satellite/google-maps-satellite.dgml")
        self.setProjection(Marble.Mercator)
        self.setShowOverviewMap(False)

        self._home_map = map_info_parser.get_default()
        self._map_coords = map_info_parser.get_gps_dict()
        def_latlonzoom = self._map_coords[self._home_map]
        self._home_pt = Marble.GeoDataCoordinates(def_latlonzoom[1], def_latlonzoom[0], 0.0, Marble.GeoDataCoordinates.Degree) # +
        self.centerOn(self._home_pt)
        self.setZoom(def_latlonzoom[2])

        paintlayer = PaintLayer(self)
        self.addLayer(paintlayer)

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
        self._home_map = map_name
        self.update()

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
