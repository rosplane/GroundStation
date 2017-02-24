from PyKDE4.marble import *
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import os.path
import math

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

        # Stationary obstacles will have a latitude, longitude, cylinder_radius and cylinder_height
        self.stationaryObstacles = [(38.147,-76.428,20,25),
        (38.148,-76.429,40,50),(38.149,-76.430,60,75),(38.150,-76.431,80,100),
        (38.152,-76.433,100,125),(38.154,-76.435,120,150)]

        # Moving obstacles will have a latitude, longitude, sphere_radius and sphere_height
        self.movingObstacles = [(38.147,-76.424,20,81),
        (38.148,-76.425,40,50),(38.149,-76.426,60,75),(38.150,-76.427,80,100),
        (38.152,-76.429,100,125),(38.154,-76.431,120,210)]

    def renderPosition(self): # ??
        return ['SURFACE']

    def render(self, painter, viewPort, renderPos, layer):
        painter.m_index = 0 # should provide a surface paint
        painter.setRenderHint(QPainter.Antialiasing, True)
        self.drawWaypoints(painter)
        # Don't draw these if it's too zoomed out
        if (self.marble.zoom() > 2700): 
            self.drawStationaryObstacles(painter)
            self.drawMovingObstacles(painter)
        return True

    def drawWaypoints(self, painter):
        painter.setPen(QPen(QBrush(Qt.red), 4.5, Qt.SolidLine, Qt.RoundCap))
        # Draw waypoints according to latlong degrees for current map
        self._home_map = self.marble._home_map
        self.waypoints = map_info_parser.get_waypoints(self._home_map)

        for waypoint in self.waypoints:
            location = Marble.GeoDataCoordinates(waypoint[1], waypoint[0], 0.0, Marble.GeoDataCoordinates.Degree)
            painter.drawEllipse(location, 5, 5)

    def drawStationaryObstacles(self, painter):
        # height and radius are both in meters (not feet!)
        UAV_height = 100 # meters
        referenceDistance = self.marble.distanceFromZoom(self.marble.zoom())*1000
        # Draw obstacles according to latlong degrees and height relative to the UAV
        for (latitude, longitude, radius, height) in self.stationaryObstacles:
            pixelDiameter = math.ceil(2*220*radius/referenceDistance)
            heightDiff = height-UAV_height
            location = Marble.GeoDataCoordinates(longitude, latitude, height, Marble.GeoDataCoordinates.Degree)
            
            painter.setPen(QPen(QBrush(Qt.red), 1, Qt.SolidLine, Qt.RoundCap))
            if (heightDiff < 0):
                painter.setPen(QPen(QBrush(Qt.darkGreen), 1, Qt.SolidLine, Qt.RoundCap))
            painter.drawEllipse(location, pixelDiameter, pixelDiameter)
            painter.drawText(location, str(height))

    def drawMovingObstacles(self, painter):
        UAV_height = 100 # meters
        referenceDistance = self.marble.distanceFromZoom(self.marble.zoom())*1000
        # Draw obstacles according to latlong degrees and height relative to the UAV
        for (latitude, longitude, radius, height) in self.movingObstacles:
            heightDiff = height-UAV_height
            location = Marble.GeoDataCoordinates(longitude, latitude, height, Marble.GeoDataCoordinates.Degree)
            
            # Draw maximum radius of sphere in grey
            pixelDiameter = math.ceil(2*220*radius/referenceDistance)
            painter.setPen(QPen(QBrush(Qt.gray), 1, Qt.SolidLine, Qt.RoundCap))
            painter.drawEllipse(location, pixelDiameter, pixelDiameter)
            painter.drawText(location, str(heightDiff)+ "("+str(radius)+")")
            
            # Draw radius of sphere at current height in red (if applicable)
            # Calculate radius of sphere at current height
            if abs(heightDiff) < radius:
                localRadius = math.sqrt(radius*radius - heightDiff*heightDiff)
                pixelDiameter = math.ceil(2*220*localRadius/referenceDistance)
                painter.setPen(QPen(QBrush(Qt.red), 1, Qt.SolidLine, Qt.RoundCap))
                painter.drawEllipse(location, pixelDiameter, pixelDiameter)


class MarbleMap(Marble.MarbleWidget):
    def __init__(self, parent=None): # Parent line VERY important
        super(MarbleMap, self).__init__() # MarbleWidget constructor
        # Check if Google Maps files exist
        if os.path.isfile("~/.local/share/marble/maps/earth/google-maps-satellite/google-maps-satellite.dgml"):
            self.setMapThemeId("earth/google-maps-satellite/google-maps-satellite.dgml")
        else: # Default to open street map
            self.setMapThemeId("earth/openstreetmap/openstreetmap.dgml") # street view
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
