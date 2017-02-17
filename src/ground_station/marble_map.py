from PyKDE4.marble import *
from PyQt4.QtCore import *
from PyQt4.QtGui import *

import map_info_parser
#import rospy
#from fcu_common.msg import FW_Waypoint
#import numpy as np

'''
For changing color of current waypoint to green:
Use the QTimer (as shown in the layer tutorial) to call the marble.update method
at every set interval [[in fact, do it like map_widget's Plane() class, but in this
MarbleMap() class]],
and have PaintLayer have an internal state-subscriber-like class whose data member
of current waypoint is always updating.
'''
# Class for allowing the widget to paint to the marble map
class PaintLayer(Marble.LayerInterface, QObject):
    def __init__(self, marble):
        QObject.__init__(self)
        self.marble = marble
        ''' This is only a temporary object; should be replaced by an xml file or something
            These are the competition waypoints, and are currently set to always be drawn
            at the specified latlong coordinates
        '''
        self.waypoints = [(38.14531389,-76.42911944),
        (38.149222,-76.4294833),(38.150133,-76.4308556),(38.14895,-76.4322861),
        (38.147011,-76.4306417),(38.1437833,-76.4319944)]

    def renderPosition(self): # ??
        return ['SURFACE']

    def render(self, painter, viewPort, renderPos, layer):
        painter.m_index = 0 # should provide a surface paint
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setPen(QPen(QBrush(Qt.red), 4.5, Qt.SolidLine, Qt.RoundCap))
        # Draw waypoints according to latlong degrees
        for waypoint in self.waypoints:
            location = Marble.GeoDataCoordinates(waypoint[1], waypoint[0], 0.0, Marble.GeoDataCoordinates.Degree)
            painter.drawEllipse(location, 5, 5)

        return True

class MarbleMap(Marble.MarbleWidget):
    def __init__(self, parent=None): # Parent line VERY important
        super(MarbleMap, self).__init__() # MarbleWidget constructor
        #self.setMapThemeId("earth/satelliteview/bluemarble.dgml") # satellite, doesn't load very well
        self.setMapThemeId("earth/openstreetmap/openstreetmap.dgml") # street view
        self.setProjection(Marble.Mercator)
        self.setShowOverviewMap(False)

        self._map_coords = map_info_parser.get_gps_dict()
        def_latlonzoom = self._map_coords[map_info_parser.get_default()]
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
