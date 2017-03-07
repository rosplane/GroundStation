from PyKDE4.marble import *
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import os.path
import math

import map_info_parser
import rospy
from fcu_common.msg import GPS, Obstacles, Obstacle

'''
For changing color of current waypoint to green:
Use the QTimer (as shown in the layer tutorial) to call the marble.update method
at every set interval [[in fact, do it like map_widget's Plane() class, but in this
MarbleMap() class]],
and have PaintLayer have an internal state-subscriber-like class whose data member
of current waypoint is always updating.
'''
class GPSSubscriber():
    def __init__(self):
        # subscribing to fcu_common/GPS to get plane latitude and longitude
        self.lat = 0.0 # in degrees
        self.lon = 0.0
        self.altitude = 0.0
        rospy.Subscriber("/gps/data", GPS, self.callback)
        
        #rospy.Subscriber("/waypoint_path", FW_Waypoint, self.callback_waypoints)

    def callback(self, gps):
        self.lat = gps.latitude
        self.lon = -1.0*gps.longitude
        self.altitude = gps.altitude

class ObstaclesSubscriber():
    def __init__(self):
        self.stationaryObstacles = []
        self.movingObstacles = []
        rospy.Subscriber("/obstacles", Obstacles, self.callback)

    def callback(self, obstacles):
        self.stationaryObstacles = []
        for obstacle in obstacles.stationary_obstacles:
            lat = obstacle.latitude
            lon = obstacle.longitude
            radius = obstacle.radius
            height = obstacle.height
            self.stationaryObstacles.append((lat, lon, radius, height))

        self.movingObstacles = []
        for obstacle in obstacles.moving_obstacles:
            lat = obstacle.latitude
            lon = obstacle.longitude
            radius = obstacle.radius
            height = obstacle.height
            self.movingObstacles.append((lat, lon, radius, height))


# Class for allowing the widget to paint to the marble map
class PaintLayer(Marble.LayerInterface, QObject):
    def __init__(self, marble):
        QObject.__init__(self)
        self.marble = marble
        self._home_map = self.marble._home_map
        self.waypoints = map_info_parser.get_waypoints(self._home_map)
        self.gpsSubscriber = GPSSubscriber()
        self.obsSubscriber = ObstaclesSubscriber()
        # subscriber class +++++++++++++++++++++++++++++++++++

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
        UAV_height = self.gpsSubscriber.altitude # meters
        referenceDistance = self.marble.distanceFromZoom(self.marble.zoom())*1000
        # Draw obstacles according to latlong degrees and height relative to the UAV
        for (latitude, longitude, radius, height) in self.obsSubscriber.stationaryObstacles:
            pixelDiameter = math.ceil(2*220*radius/referenceDistance)
            heightDiff = height-UAV_height
            location = Marble.GeoDataCoordinates(longitude, latitude, height, Marble.GeoDataCoordinates.Degree)
            
            painter.setPen(QPen(QBrush(Qt.red), pixelDiameter/2, Qt.SolidLine, Qt.RoundCap))
            if (heightDiff < 0):
                painter.setPen(QPen(QBrush(Qt.darkGreen), pixelDiameter/2, Qt.SolidLine, Qt.RoundCap))
            painter.drawEllipse(location, pixelDiameter/2, pixelDiameter/2)
            painter.setPen(QPen(QBrush(Qt.black), 1, Qt.SolidLine, Qt.RoundCap))
            painter.drawText(location, str(math.floor(heightDiff)))

    def drawMovingObstacles(self, painter):
        UAV_height = self.gpsSubscriber.altitude # meters
        referenceDistance = self.marble.distanceFromZoom(self.marble.zoom())*1000
        # Draw obstacles according to latlong degrees and height relative to the UAV
        for (latitude, longitude, radius, height) in self.obsSubscriber.movingObstacles:
            heightDiff = height-UAV_height
            location = Marble.GeoDataCoordinates(longitude, latitude, height, Marble.GeoDataCoordinates.Degree)
            
            # Draw maximum radius of sphere in grey
            pixelDiameter = math.ceil(2*220*radius/referenceDistance)
            painter.setPen(QPen(QBrush(Qt.darkGreen), pixelDiameter/2, Qt.SolidLine, Qt.RoundCap))
            painter.drawEllipse(location, pixelDiameter/2, pixelDiameter/2)
            
            # Draw radius of sphere at current height in red (if applicable)
            # Calculate radius of sphere at current height
            if abs(heightDiff) < radius:
                localRadius = math.sqrt(radius*radius - heightDiff*heightDiff)
                pixelDiameter = math.ceil(2*220*localRadius/referenceDistance)
                painter.setPen(QPen(QBrush(Qt.red), pixelDiameter/2, Qt.SolidLine, Qt.RoundCap))
                painter.drawEllipse(location, pixelDiameter/2, pixelDiameter/2)
            painter.setPen(QPen(QBrush(Qt.black), 1, Qt.SolidLine, Qt.RoundCap))
            painter.drawText(location, str(math.floor(heightDiff))+ "("+str(math.floor(radius))+")")


class MarbleMap(Marble.MarbleWidget):
    def __init__(self, parent=None): # Parent line VERY important
        super(MarbleMap, self).__init__() # MarbleWidget constructor
        # Check if Google Maps files exist
        googleMapsPath = os.path.expanduser("~/.local/share/marble/maps/earth/google-maps-satellite/")
        if os.path.exists(googleMapsPath):
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
