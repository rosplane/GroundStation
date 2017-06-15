import xml.etree.cElementTree as ET
import os, errno

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

def get_latlon(map_name):
    xmlroot = ET.parse(INFO_FILE_PATH).getroot()
    mapnode = [ node for node in xmlroot.findall('map') if node.attrib['name'] == map_name ][0]
    if (not mapnode.find('start_lat') is None) and (not mapnode.find('start_lon')):
        lat = float(str(mapnode.find('start_lat').text))
        lon = float(str(mapnode.find('start_lon').text))
    else:
        lat = float(str(mapnode.find('lat').text))
        lon = float(str(mapnode.find('lon').text))
    return [lat, lon]

def get_waypoints(wp_file_path):
    if os.path.exists(wp_file_path):
        wp_list = []
        with open(wp_file_path, 'r') as wp_file:
            for line in wp_file:
                wp_t = line.split()
                lat = float(wp_t[0])
                lon = float(wp_t[1])
                alt = float(wp_t[2])
                chi = float(wp_t[3])
                wp_list.append((lat, lon, alt, chi))
        return wp_list
    else:
        # make a blank file and directory, if necessary
        try:
            os.makedirs(os.path.dirname(wp_file_path))
        except OSError as exc:
            if exc.errno != errno.EEXIST:
                raise
        open(wp_file_path, 'a').close()
        return []

def get_typed_waypoints(map_name, folder_name):
    wp_file_path = os.path.join(PWD,'resources','wp_data',folder_name,'%s_%s.txt' % (map_name, folder_name))
    return get_waypoints(wp_file_path)

def get_windspeed_components():
    ws_file_path = os.path.join(PWD, 'resources', 'wp_data', 'drop_wps', '_WINDSPEED_.txt')
    try:
        with open(ws_file_path, 'r') as ws_file:
            ws_t = ws_file.read().split()
            Vwind_n = float(ws_t[0])
            Vwind_e = float(ws_t[1])
        return [Vwind_n, Vwind_e]
    except:
        return [0.0, 0.0]
