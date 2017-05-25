# Taylor Pool
# March 14, 2017
# AUVSI Project
# Contains a Geobase class that can be used to convert from GPS to NED and vise versa

##############################


from geographiclib.geodesic import Geodesic
import math

class Geobase:
    #Initializer Function
    def __init__(self, originLat, originLong, originHeight=0):  #Sets base location
        self.origin = [originLat, originLong, originHeight]

    #Instance Base Location Modifier
    def change_origin(self, originLat, originLong, originHeight=0):
        self.origin = [originLat, originLong, originHeight]

    #GPS to NED
    #Pre: lat2 and long2 are in long decimal format or DD-MM-SS
    #Post: Returns a list containing location in [north, east, down]
    def gps_to_ned(self, lat2, long2, height=0):
        #Conversion to Long Decimal Format from DD-MM-SS
        values = [str(lat2), str(long2)]
        newValues = []
        for value in values:
            if ("N" in value) or ("S" in value) or ("E" in value) or ("W" in value) == True:
                newValues.append(decimal_degrees(value))
            else:
                newValues.append(float(value))
        diction = Geodesic.WGS84.Inverse(self.origin[0], self.origin[1], newValues[0], newValues[1])
        solution = [diction['s12']*math.cos(math.radians(diction['azi1'])), diction['s12']*math.sin(math.radians(diction['azi1'])), -height]
        return solution

    #NED to GPS
    #Pre: north, east ,down are in meters
    #Post: Returns a list containing GPS [latitude, longitude, altitude]
    def ned_to_gps(self, north, east, down = 0):
        s_12 = math.sqrt(north**2+east**2)
        azi_1 = math.degrees(math.asin(east/s_12))
        diction = Geodesic.WGS84.Direct(self.origin[0], self.origin[1], azi_1, s_12)
        solution = [diction['lat2'], diction['lon2'], -down]
        return solution

    #Function decimal_degrees
    #Pre: string is in format [N/E or S/W]DD-MM-SS.SS
    #Post: Returns GPS component in long decimal format
    def decimal_degrees (string):
        a = 0
        firstLetter = string[0]
        if firstLetter == 'N' or firstLetter == 'E':
            a = 1
        elif firstLetter == 'S' or firstLetter == 'W':
            a = -1
        lessString = string.strip("NSEW ")
        values = lessString.split('-', 2)

        d = float(values[0])
        m = float(values[1])
        s = float(values[2])

        decimal = a*(d+(m/60.0)+(s/3600.0))

        return decimal
