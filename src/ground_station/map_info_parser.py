import xml.etree.cElementTree as ET
import os

PWD = os.path.dirname(os.path.abspath(__file__))
INFO_FILE_PATH = os.path.join(PWD, 'resources', 'map_info.xml')

def get_default():
    xmlroot = ET.parse(INFO_FILE_PATH).getroot()
    return xmlroot.attrib['default']

def get_gps_dict():
    gps_dict = {}
    xmlroot = ET.parse(INFO_FILE_PATH).getroot()
    for xmlnode in xmlroot.findall('map'):
        name = xmlnode.attrib['name']
        lat = float(str(xmlnode.find('lat').text))
        lon = float(str(xmlnode.find('lon').text))
        zoom = int(str(xmlnode.find('zoom').text))
        gps_dict[name] = [lat, lon, zoom]
    return gps_dict

def get_waypoints(map_name):
    xmlroot = ET.parse(INFO_FILE_PATH).getroot()
    mapnode = [ node for node in xmlroot.findall('map') if node.attrib['name'] == map_name ][0]
    if not mapnode.find('wp') is None:
        return eval(mapnode.find('wp').text.strip())
    return []
