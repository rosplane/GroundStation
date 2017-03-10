from PyKDE4.marble import *
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import os.path
from math import ceil, floor, sqrt, sin, asin, cos, acos, radians, degrees

import map_info_parser
import rospy
from fcu_common.msg import FW_State, GPS, Obstacles, Obstacle

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

class StateSubscriber(): # For rendering rotated plane onto marble widget
    def __init__(self):
        self.pe = 0.0
        self.pn = 0.0
        self.psi = 0.0
        rospy.Subscriber("/junker/truth", FW_State, self.callback)

    def callback(self, state):
        self.pe = state.position[1]
        self.pn = state.position[0]
        self.psi = state.psi

# Class for allowing the widget to paint to the marble map
class PaintLayer(Marble.LayerInterface, QObject):
    def __init__(self, marble):
        QObject.__init__(self)
        self.marble = marble
        self._home_map = self.marble._home_map
        self.waypoints = map_info_parser.get_waypoints(self._home_map)

        self.gpsSubscriber = GPSSubscriber()
        self.obsSubscriber = ObstaclesSubscriber()
        self.stateSubscriber = StateSubscriber()

        # For meters to GPS conversion and plane geometry
        self.latlon = map_info_parser.get_latlon(self._home_map)
        self.R = 6371000.0           # Radius of earth in meters
        self.R_prime = cos(radians(self.latlon[0]))*self.R
        self.h = 20
        self.w = 20

    def deToLat(self, de):
        return self.latlon[0] + degrees(asin(de/self.R))

    def dnToLon(self, dn):
        return self.latlon[1] + degrees(asin(dn/self.R_prime))

    def renderPosition(self): # So that Marble knows where to paint
        return ['SURFACE']

    def render(self, painter, viewPort, renderPos, layer):
        painter.m_index = 0 # should provide a surface paint
        painter.setRenderHint(QPainter.Antialiasing, True)
        self._home_map = self.marble._home_map
        self.drawWaypoints(painter)
        # Don't draw these if it's too zoomed out
        if (self.marble.zoom() > 2700):
            self.drawStationaryObstacles(painter)
            self.drawMovingObstacles(painter)
        self.drawPlane(painter) # Plane on top of all other items in drawing
        return True

    def rotate_x(self, x, y, a):
        return x * cos(a) + y * sin(a)

    def rotate_y(self, x, y, a):
        return -1 * x * sin(a) + y * cos(a)

    def drawPlane(self, painter):
        painter.setPen(QPen(QBrush(Qt.black), 3.5, Qt.SolidLine, Qt.RoundCap))
        self.latlon = map_info_parser.get_latlon(self._home_map)
        self.R_prime = cos(radians(self.latlon[0]))*self.R
        de = self.stateSubscriber.pe
        dn = self.stateSubscriber.pn
        psi = self.stateSubscriber.psi

        # Draw Plane Lines with pts 1-7
        referenceDistance = self.marble.distanceFromZoom(self.marble.zoom())
        scaled_h = ceil(6*self.h*referenceDistance)
        scaled_w = ceil(6*self.w*referenceDistance)
        #pt_1 = [de, dn + scaled_h/2]
        #pt_2 = [de, dn - scaled_h/2]
        pt_1 = [de + self.rotate_x(0, scaled_h/2, psi), dn + self.rotate_y(0, scaled_h/2, psi)]
        pt_2 = [de + self.rotate_x(0, -scaled_h/2, psi), dn + self.rotate_y(0, -scaled_h/2, psi)]
        line_1 = Marble.GeoDataLineString()
        line_1.append(Marble.GeoDataCoordinates(self.dnToLon(pt_1[1]), self.deToLat(pt_1[0]), 0.0, Marble.GeoDataCoordinates.Degree))
        line_1.append(Marble.GeoDataCoordinates(self.dnToLon(pt_2[1]), self.deToLat(pt_2[0]), 0.0, Marble.GeoDataCoordinates.Degree))
        pt_3 = [de, dn]
        #pt_4 = [de - scaled_w/2, dn - scaled_h/4]
        #pt_5 = [de + scaled_w/2, dn - scaled_h/4]
        pt_4 = [de + self.rotate_x(-scaled_w/2, -scaled_h/4, psi), dn + self.rotate_y(-scaled_w/2, -scaled_h/4, psi)]
        pt_5 = [de + self.rotate_x(scaled_w/2, -scaled_h/4, psi), dn + self.rotate_y(scaled_w/2, -scaled_h/4, psi)]
        line_2 = Marble.GeoDataLineString()
        line_2.append(Marble.GeoDataCoordinates(self.dnToLon(pt_3[1]), self.deToLat(pt_3[0]), 0.0, Marble.GeoDataCoordinates.Degree))
        line_2.append(Marble.GeoDataCoordinates(self.dnToLon(pt_4[1]), self.deToLat(pt_4[0]), 0.0, Marble.GeoDataCoordinates.Degree))
        line_3 = Marble.GeoDataLineString()
        line_3.append(Marble.GeoDataCoordinates(self.dnToLon(pt_3[1]), self.deToLat(pt_3[0]), 0.0, Marble.GeoDataCoordinates.Degree))
        line_3.append(Marble.GeoDataCoordinates(self.dnToLon(pt_5[1]), self.deToLat(pt_5[0]), 0.0, Marble.GeoDataCoordinates.Degree))
        #pt_6 = [de - scaled_w/4, dn - 2*scaled_h/5]
        #pt_7 = [de + scaled_w/4, dn - 2*scaled_h/5]
        pt_6 = [de + self.rotate_x(-scaled_w/4, -2*scaled_h/5, psi), dn + self.rotate_y(-scaled_w/4, -2*scaled_h/5, psi)]
        pt_7 = [de + self.rotate_x(scaled_w/4, -2*scaled_h/5, psi), dn + self.rotate_y(scaled_w/4, -2*scaled_h/5, psi)]
        line_4 = Marble.GeoDataLineString()
        line_4.append(Marble.GeoDataCoordinates(self.dnToLon(pt_6[1]), self.deToLat(pt_6[0]), 0.0, Marble.GeoDataCoordinates.Degree))
        line_4.append(Marble.GeoDataCoordinates(self.dnToLon(pt_7[1]), self.deToLat(pt_7[0]), 0.0, Marble.GeoDataCoordinates.Degree))

        painter.drawPolyline(line_1)
        painter.drawPolyline(line_2)
        painter.drawPolyline(line_3)
        painter.drawPolyline(line_4)

    def drawWaypoints(self, painter):
        painter.setPen(QPen(QBrush(Qt.red), 4.5, Qt.SolidLine, Qt.RoundCap))
        # Draw waypoints according to latlong degrees for current map
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
            pixelDiameter = ceil(2*220*radius/referenceDistance)
            heightDiff = height-UAV_height
            location = Marble.GeoDataCoordinates(longitude, latitude, height, Marble.GeoDataCoordinates.Degree)

            painter.setPen(QPen(QBrush(Qt.red), pixelDiameter/2, Qt.SolidLine, Qt.RoundCap))
            if (heightDiff < 0):
                painter.setPen(QPen(QBrush(Qt.darkGreen), pixelDiameter/2, Qt.SolidLine, Qt.RoundCap))
            painter.drawEllipse(location, pixelDiameter/2, pixelDiameter/2)
            painter.setPen(QPen(QBrush(Qt.black), 1, Qt.SolidLine, Qt.RoundCap))
            painter.drawText(location, str(floor(heightDiff)))

    def drawMovingObstacles(self, painter):
        UAV_height = self.gpsSubscriber.altitude # meters
        referenceDistance = self.marble.distanceFromZoom(self.marble.zoom())*1000
        # Draw obstacles according to latlong degrees and height relative to the UAV
        for (latitude, longitude, radius, height) in self.obsSubscriber.movingObstacles:
            heightDiff = height-UAV_height
            location = Marble.GeoDataCoordinates(longitude, latitude, height, Marble.GeoDataCoordinates.Degree)

            # Draw maximum radius of sphere in grey
            pixelDiameter = ceil(2*220*radius/referenceDistance)
            painter.setPen(QPen(QBrush(Qt.darkGreen), pixelDiameter/2, Qt.SolidLine, Qt.RoundCap))
            painter.drawEllipse(location, pixelDiameter/2, pixelDiameter/2)

            # Draw radius of sphere at current height in red (if applicable)
            # Calculate radius of sphere at current height
            if abs(heightDiff) < radius:
                localRadius = sqrt(radius*radius - heightDiff*heightDiff)
                pixelDiameter = ceil(2*220*localRadius/referenceDistance)
                painter.setPen(QPen(QBrush(Qt.red), pixelDiameter/2, Qt.SolidLine, Qt.RoundCap))
                painter.drawEllipse(location, pixelDiameter/2, pixelDiameter/2)
            painter.setPen(QPen(QBrush(Qt.black), 1, Qt.SolidLine, Qt.RoundCap))
            painter.drawText(location, str(floor(heightDiff))+ "("+str(floor(radius))+")")


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

    def change_home(self, map_name):
        latlonzoom = self._map_coords[map_name]
        self._home_pt = Marble.GeoDataCoordinates(latlonzoom[1], latlonzoom[0], 0.0, Marble.GeoDataCoordinates.Degree)
        self.centerOn(self._home_pt)
        self.setZoom(latlonzoom[2])
        self._home_map = map_name
        self.update()

    def get_home(self):
        return self._home_pt
