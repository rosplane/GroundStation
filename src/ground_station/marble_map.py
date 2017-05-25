from PyKDE4.marble import *
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import os.path
from math import ceil, floor, sqrt, sin, asin, cos, acos, radians, degrees, fmod, pi

import map_info_parser
import rospy
from std_msgs.msg import String
from fcu_common.msg import State, GPS, RCRaw
from ros_plane.msg import Current_Path, Waypoint
from Signals import WP_Handler
from .Geo import Geobase
import json, re, math

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

class MissionSubscriber():
    def __init__(self):
        self.mission_data = ""
        rospy.Subscriber("missions", String, self.callback)

    def callback(self, mission_json):
        json_data = mission_json.data
        json_data = re.sub(r"u'",r'"',json_data)
        json_data = re.sub(r"'",r'"',json_data)
        json_data = re.sub(r"True",r'"True"',json_data)
        json_data = re.sub(r"False",r'"False"',json_data)
        data = json.loads(json_data)[0]
        self.mission_data = data


class ObstaclesSubscriber():
    def __init__(self):
        self.stationaryObstacles = []
        self.movingObstacles = []
        rospy.Subscriber("/obstacles", String, self.json_callback)

    def json_callback(self, obstacles_json):
        json_data = obstacles_json.data
        json_data = re.sub(r"u'",r'"',json_data)
        json_data = re.sub(r"'",r'"',json_data)
        data = json.loads(json_data)
        moving_obstacles = data["moving_obstacles"]
        stationary_obstacles = data["stationary_obstacles"]

        self.movingObstacles = []
        for obstacle in moving_obstacles:
            lat = float(obstacle["latitude"])
            lon = float(obstacle["longitude"])
            radius = float(obstacle["sphere_radius"])
            height = float(obstacle["altitude_msl"])
            self.movingObstacles.append((lat, lon, radius, height))

        self.stationaryObstacles = []
        for obstacle in stationary_obstacles:
            lat = float(obstacle["latitude"])
            lon = float(obstacle["longitude"])
            radius = float(obstacle["cylinder_radius"])
            height = float(obstacle["cylinder_height"])
            self.stationaryObstacles.append((lat, lon, radius, height))

class StateSubscriber(): # For rendering rotated plane onto marble widget
    def __init__(self):
        self.pe = 0.0
        self.pn = 0.0
        self.psi = 0.0

        self.lat = 0
        self.lon = 0
        self.alt_ft = 0

        rospy.Subscriber("/junker/truth", State, self.callback)
        rospy.Subscriber("/state", State, self.callback)
        rospy.Subscriber("/gps_state", State, self.gpsStateCallback)

    def callback(self, state):
        self.pe = state.position[1]
        self.pn = state.position[0]
        self.psi = fmod(state.chi, 2*pi)

    def gpsStateCallback(self, state):
        self.lat = state.position[0]
        self.lon = state.position[1]
        self.alt_ft = state.position[2]

class MiscSubscriber():
    def __init__(self):
        rospy.Subscriber("/current_path", Current_Path, self.callback_curPath)
        rospy.Subscriber("/rc_raw", RCRaw, self.callback_RC)
        rospy.Subscriber("/waypoint_path", Waypoint, self.callback_waypoints)
        rospy.Subscriber("/gps/data", GPS, self.callback_GPS)

        self.autopilotEnabled = False
        self.curPath = Current_Path()
        self.waypoints = []
        self.curPath.flag = False
        self.numSat = 0

    def callback_curPath(self, cp):
        self.curPath = cp

    def callback_RC(self, rcRaw):
        self.autopilotEnabled = (rcRaw.values[4] < 1700)

    def callback_waypoints(self, waypoint):
        self.waypoints.append(waypoint)

    def callback_GPS(self, gps_data):
        self.numSat = gps_data.NumSat
        
        


# Class for allowing the widget to paint to the marble map
class PaintLayer(Marble.LayerInterface, QObject):
    def __init__(self, marble):
        QObject.__init__(self)
        self.marble = marble
        self._home_map = self.marble._home_map
        # This list must be kept in sync with the plane's on-board waypoint queue
        self.waypoints = map_info_parser.get_waypoints(self._home_map)

        self.gpsSubscriber = GPSSubscriber()
        self.obsSubscriber = ObstaclesSubscriber()
        self.missionSubscriber = MissionSubscriber()
        self.stateSubscriber = StateSubscriber()
        self.miscSubscriber = MiscSubscriber()


        # For meters to GPS conversion and plane geometry
        # specifically starting lat, lon of the plane
        self.latlon = self.marble.latlon
        self.R = 6371000.0           # Radius of earth in meters
        self.R_prime = cos(radians(self.latlon[0]))*self.R
        self.h = 20
        self.w = 20

        # For signal handling
        self.marble.WPH.wp_inserted.connect(self.add_waypoint)
        self.marble.WPH.wp_removed.connect(self.remove_waypoint)
        self.marble.WPH.home_changed.connect(self.change_home)

    def add_waypoint(self, lat, lon, alt, pos):
        self.waypoints.insert(pos, (lat, lon, alt))

    def remove_waypoint(self, pos):
        del self.waypoints[pos]

    def change_home(self, new_home):
        self.waypoints = map_info_parser.get_waypoints(new_home)
        self.latlon = self.marble.latlon
        self.R_prime = cos(radians(self.latlon[0]))*self.R
        self._home_map = new_home

    def dnToLat(self, dn):
        return self.latlon[0] + degrees(asin(dn/self.R))

    def deToLon(self, de):
        return self.latlon[1] + degrees(asin(de/self.R_prime))

    def renderPosition(self): # So that Marble knows where to paint
        return ['SURFACE']

    def render(self, painter, viewPort, renderPos, layer):
        painter.m_index = 0 # should provide a surface paint
        painter.setRenderHint(QPainter.Antialiasing, True)
        #self._home_map = self.marble._home_map
        self.drawWaypoints(painter)
        # Don't draw these if it's too zoomed out
        if (self.marble.zoom() > 2700):
            self.drawStationaryObstacles(painter)
            self.drawMovingObstacles(painter)
            self.drawMissionDetails(painter)
        self.drawCurPath(painter)
        self.drawPlane(painter) # Plane on top of all other items in drawing
        # self.drawHomePoint(painter)
        # self.drawGPSDot(painter)
        return True

    def rotate_x(self, x, y, a):
        return x * cos(a) + y * sin(a)

    def rotate_y(self, x, y, a):
        return -1 * x * sin(a) + y * cos(a)

    def drawHomePoint(self, painter):
        painter.setPen(QPen(QBrush(Qt.green), 4.5, Qt.SolidLine, Qt.RoundCap))
        location = Marble.GeoDataCoordinates(self.deToLon(0), self.dnToLat(0), 0.0, Marble.GeoDataCoordinates.Degree)
        painter.drawEllipse(location, 5, 5)


    def drawCurPath(self, painter):
        painter.setPen(QPen(QBrush(Qt.red), 3.5, Qt.SolidLine, Qt.RoundCap))
        curPath = self.miscSubscriber.curPath
        # Straight line
        if curPath.flag == True:
            r = curPath.r
            q = curPath.q
            scale = 200
            pt_1 = [r[1],r[0]]
            pt_2 = [r[1]+scale*q[1],r[0]+scale*q[0]]
            line_1 = Marble.GeoDataLineString()
            line_1.append(Marble.GeoDataCoordinates(self.deToLon(pt_1[0]), self.dnToLat(pt_1[1]), 0.0, Marble.GeoDataCoordinates.Degree))
            line_1.append(Marble.GeoDataCoordinates(self.deToLon(pt_2[0]), self.dnToLat(pt_2[1]), 0.0, Marble.GeoDataCoordinates.Degree))
            painter.drawPolyline(line_1)
        else:
            # Loiter
            c = curPath.c 
            R = curPath.rho
            referenceDistance = self.marble.distanceFromZoom(self.marble.zoom())*1000
            location = Marble.GeoDataCoordinates(self.deToLon(c[1]), self.dnToLat(c[0]), 0.0, Marble.GeoDataCoordinates.Degree)
            pixelRadius = ceil(6.8*67*R/referenceDistance)
            painter.drawEllipse(location, pixelRadius, pixelRadius)

    def drawGPSDot(self, painter):
        lon, lat, alt_ft = self.stateSubscriber.lon, self.stateSubscriber.lat, self.stateSubscriber.alt_ft
        painter.setPen(QPen(QBrush(Qt.red), 4.5, Qt.SolidLine, Qt.RoundCap))
        location = Marble.GeoDataCoordinates(lon, lat, 0.0, Marble.GeoDataCoordinates.Degree)
        painter.drawEllipse(location, 5, 5)



    def drawPlane(self, painter):
        autopilotEnabled = self.miscSubscriber.autopilotEnabled
        if autopilotEnabled:
            painter.setPen(QPen(QBrush(Qt.red), 3.5, Qt.SolidLine, Qt.RoundCap))
        else:
            painter.setPen(QPen(QBrush(Qt.black), 3.5, Qt.SolidLine, Qt.RoundCap))
        #self.latlon = map_info_parser.get_latlon(self._home_map)
        #self.R_prime = cos(radians(self.latlon[0]))*self.R
        de = self.stateSubscriber.pe
        dn = self.stateSubscriber.pn
        #print((de, dn, self.latlon[0], self.latlon[1], self._home_map))#----------------
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
        line_1.append(Marble.GeoDataCoordinates(self.deToLon(pt_1[0]), self.dnToLat(pt_1[1]), 0.0, Marble.GeoDataCoordinates.Degree))
        line_1.append(Marble.GeoDataCoordinates(self.deToLon(pt_2[0]), self.dnToLat(pt_2[1]), 0.0, Marble.GeoDataCoordinates.Degree))
        pt_3 = [de, dn]
        #pt_4 = [de - scaled_w/2, dn - scaled_h/4]
        #pt_5 = [de + scaled_w/2, dn - scaled_h/4]
        pt_4 = [de + self.rotate_x(-scaled_w/2, -scaled_h/4, psi), dn + self.rotate_y(-scaled_w/2, -scaled_h/4, psi)]
        pt_5 = [de + self.rotate_x(scaled_w/2, -scaled_h/4, psi), dn + self.rotate_y(scaled_w/2, -scaled_h/4, psi)]
        line_2 = Marble.GeoDataLineString()
        line_2.append(Marble.GeoDataCoordinates(self.deToLon(pt_3[0]), self.dnToLat(pt_3[1]), 0.0, Marble.GeoDataCoordinates.Degree))
        line_2.append(Marble.GeoDataCoordinates(self.deToLon(pt_4[0]), self.dnToLat(pt_4[1]), 0.0, Marble.GeoDataCoordinates.Degree))
        line_3 = Marble.GeoDataLineString()
        line_3.append(Marble.GeoDataCoordinates(self.deToLon(pt_3[0]), self.dnToLat(pt_3[1]), 0.0, Marble.GeoDataCoordinates.Degree))
        line_3.append(Marble.GeoDataCoordinates(self.deToLon(pt_5[0]), self.dnToLat(pt_5[1]), 0.0, Marble.GeoDataCoordinates.Degree))
        #pt_6 = [de - scaled_w/4, dn - 2*scaled_h/5]
        #pt_7 = [de + scaled_w/4, dn - 2*scaled_h/5]
        pt_6 = [de + self.rotate_x(-scaled_w/4, -2*scaled_h/5, psi), dn + self.rotate_y(-scaled_w/4, -2*scaled_h/5, psi)]
        pt_7 = [de + self.rotate_x(scaled_w/4, -2*scaled_h/5, psi), dn + self.rotate_y(scaled_w/4, -2*scaled_h/5, psi)]
        line_4 = Marble.GeoDataLineString()
        line_4.append(Marble.GeoDataCoordinates(self.deToLon(pt_6[0]), self.dnToLat(pt_6[1]), 0.0, Marble.GeoDataCoordinates.Degree))
        line_4.append(Marble.GeoDataCoordinates(self.deToLon(pt_7[0]), self.dnToLat(pt_7[1]), 0.0, Marble.GeoDataCoordinates.Degree))

        painter.drawPolyline(line_1)
        painter.drawPolyline(line_2)
        painter.drawPolyline(line_3)
        painter.drawPolyline(line_4)

    def drawMissionDetails(self, painter):
        mission_data = self.missionSubscriber.mission_data

        # mission_data = {"fly_zones": [
        #     {
        #         "altitude_msl_max": 200.0,
        #         "altitude_msl_min": 100.0,
        #         "boundary_pts": [
        #             {
        #                 "latitude": 40.1765598495,
        #                 "longitude": -111.655536016,
        #                 "order": 1
        #             },
        #             {
        #                 "latitude": 40.176478653,
        #                 "longitude": -111.650451777,
        #                 "order": 2
        #             },
        #             {
        #                 "latitude": 40.1726680255,
        #                 "longitude": -111.647673027,
        #                 "order": 3
        #             },
        #             {
        #                 "latitude": 40.1726680255,
        #                 "longitude": -111.655419845,
        #                 "order": 4
        #             }
        #         ]
        #     }
        # ]}

        if len(mission_data) == 0:
            return

        painter.setPen(QPen(QBrush(Qt.blue), 4.5, Qt.SolidLine, Qt.RoundCap))

        # Draw boundaries
        for zone in mission_data['fly_zones']:
            line = Marble.GeoDataLineString()
            for point in zone['boundary_pts']:
                lat = point['latitude']
                lon = point['longitude']
                order = int(point['order'])
                location = Marble.GeoDataCoordinates(lon, lat, 0.0, Marble.GeoDataCoordinates.Degree)
                line.append(location)
            line.append(line[0]) # Close the polygon by adding the first point again
            painter.drawPolyline(line)


    def drawWaypoints(self, painter):

        waypointSources = ['interop','plane','internal']
        waypointsSource = waypointSources[0]

        waypoints = []
        if waypointsSource == 'interop':
            # Subscribe to waypoints published by the interop server
            waypoints = self.waypoints
        if waypointsSource == 'plane':
            # Subcribe to waypoints published by the plane
            for waypoint in self.miscSubscriber.waypoints:
                waypoints.append(waypoint.w[0],waypoint.w[1])
        if waypointsSource == 'internal':
            # Draw waypoints defined below
            waypoints = [
                [30, 0, -60],
                [180, 0, -60],
                [50, -150, -60],
                [180, 10, -60],
            ]

        painter.setPen(QPen(QBrush(Qt.red), 4.5, Qt.SolidLine, Qt.RoundCap))
        for waypoint in waypoints:
            location = Marble.GeoDataCoordinates(self.deToLon(waypoint[1]), self.dnToLat(waypoint[0]), 0.0, Marble.GeoDataCoordinates.Degree)
            painter.drawEllipse(location, 5, 5)

    def metersToFeet(self, meters):
        return meters*3.28084

    def drawStationaryObstacles(self, painter):
        # height and radius are both in feet
        UAV_height = self.metersToFeet(self.gpsSubscriber.altitude)
        referenceDistance = self.marble.distanceFromZoom(self.marble.zoom())*1000
        # Draw obstacles according to latlong degrees and height relative to the UAV
        for (latitude, longitude, radius, height) in self.obsSubscriber.stationaryObstacles:
            pixelDiameter = ceil(2*67*radius/referenceDistance)
            heightDiff = height-UAV_height
            location = Marble.GeoDataCoordinates(longitude, latitude, height, Marble.GeoDataCoordinates.Degree)

            painter.setPen(QPen(QBrush(Qt.red), pixelDiameter/2, Qt.SolidLine, Qt.RoundCap))
            if (heightDiff < 0):
                painter.setPen(QPen(QBrush(Qt.darkGreen), pixelDiameter/2, Qt.SolidLine, Qt.RoundCap))
            painter.drawEllipse(location, pixelDiameter/2, pixelDiameter/2)
            painter.setPen(QPen(QBrush(Qt.black), 1, Qt.SolidLine, Qt.RoundCap))
            painter.drawText(location, str(floor(heightDiff)))

    def drawMovingObstacles(self, painter):
        UAV_height = self.metersToFeet(self.gpsSubscriber.altitude)
        referenceDistance = self.marble.distanceFromZoom(self.marble.zoom())*1000
        # Draw obstacles according to latlong degrees and height relative to the UAV
        for (latitude, longitude, radius, height) in self.obsSubscriber.movingObstacles:
            heightDiff = height-UAV_height
            location = Marble.GeoDataCoordinates(longitude, latitude, height, Marble.GeoDataCoordinates.Degree)

            # Draw maximum radius of sphere in grey
            pixelDiameter = ceil(2*67*radius/referenceDistance)
            painter.setPen(QPen(QBrush(Qt.darkGreen), pixelDiameter/2, Qt.SolidLine, Qt.RoundCap))
            painter.drawEllipse(location, pixelDiameter/2, pixelDiameter/2)

            # Draw radius of sphere at current height in red (if applicable)
            # Calculate radius of sphere at current height
            if abs(heightDiff) < radius:
                localRadius = sqrt(radius*radius - heightDiff*heightDiff)
                pixelDiameter = ceil(2*67*localRadius/referenceDistance)
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

        self.WPH = WP_Handler()
        # For waypoint conversion
        self._home_map = map_info_parser.get_default()
        self.latlon = map_info_parser.get_latlon(self._home_map)
        self.GB = Geobase(self.latlon[0], self.latlon[1])

        self._map_coords = map_info_parser.get_gps_dict()
        def_latlonzoom = self._map_coords[self._home_map]
        self._home_pt = Marble.GeoDataCoordinates(def_latlonzoom[1], def_latlonzoom[0], 0.0, Marble.GeoDataCoordinates.Degree) # +
        self.centerOn(self._home_pt)
        self.setZoom(def_latlonzoom[2])
        self._mouse_attentive = False
        paintlayer = PaintLayer(self)
        self.addLayer(paintlayer)
        self.seconds_tests = [2.0/3600, 5.0/3600, 15.0/3600, 30.0/3600, 60.0/3600]
        self.num_s_tests = len(self.seconds_tests)

    def mousePressEvent(self, QMouseEvent): # only use if popup window is open===============
        if self._mouse_attentive:
            q_mouse_pos = QMouseEvent.pos()
            q_mouse_x = q_mouse_pos.x()
            q_mouse_y = q_mouse_pos.y()

            frame_geom = self.frameGeometry().getCoords()
            w_width = frame_geom[2] - frame_geom[0]
            w_height = frame_geom[3] - frame_geom[1]

            lat = self.centerLatitude()
            lon = self.centerLongitude()
            # Can check 2, 5, 15, 30 60 seconds, use first one that has a pixel difference from
            # center greater than 15 px with screenCoordinates()
            x = 0.0
            y = 0.0
            i = 0
            found = False

            while not found and i < self.num_s_tests:
                coord_tuple = self.screenCoordinates(lon + self.seconds_tests[i],
                                                     lat + self.seconds_tests[i])
                found = coord_tuple[0]                          # Check if too far
                if (abs(coord_tuple[1] - w_width/2) < 10):      # Check if too close
                    found = False
                if found:
                    x = abs(coord_tuple[1] - w_width/2)
                    y = abs(coord_tuple[2] - w_height/2)
                else:
                    i += 1

            if found: # Compute latlon from pixel
                x_offset = q_mouse_x - w_width / 2 # number of pixels to the east of center
                y_offset = w_height / 2 - q_mouse_y # number of pixels to the north of center
                clicked_lon = lon + x_offset*self.seconds_tests[i]/x
                clicked_lat = lat + y_offset*self.seconds_tests[i]/y
                self.WPH.emit_clicked(clicked_lat, clicked_lon)
            else: # Do nothing ===========================
                print 'Not found. Zoom in!'

    def change_home(self, map_name):
        self._home_map = map_name
        latlonzoom = self._map_coords[self._home_map]
        self._home_pt = Marble.GeoDataCoordinates(latlonzoom[1], latlonzoom[0], 0.0, Marble.GeoDataCoordinates.Degree)
        self.latlon = map_info_parser.get_latlon(self._home_map)
        self.GB = Geobase(self.latlon[0], self.latlon[1])
        self.centerOn(self._home_pt)
        self.setZoom(latlonzoom[2])
        self.update()
        self.WPH.emit_home_change(self._home_map)

    def get_home(self):
        return self._home_pt
