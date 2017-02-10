from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from PyKDE4.marble import *

from .marble_map import MarbleMap
import map_info_parser
import os

from PyQt4.QtGui import *
from PyQt4.QtCore import *
from python_qt_binding.QtGui import QIcon, QPixmap
import rospy
from fcu_common.msg import FW_State
from math import *

PWD = os.path.dirname(os.path.abspath(__file__))

class StateSubscriber(): #+++++++++++++++++++++
    def __init__(self):
        # starting with just position
        self.dn = 0.0 # "North" degrees
        self.de = 0.0 # "East" degrees
        rospy.Subscriber("/junker/truth", FW_State, self.callback)

    def callback(self, FW_State):
        # for now, conversion is found in Plane::iterate() +++++++++++++++++++++
        self.dn = FW_State.position[0]
        self.de = FW_State.position[1]

class Plane(QObject):
    def __init__(self, location):
        QObject.__init__(self)
        self.location = location
        self.timer = QTimer(self)
        self.state = StateSubscriber() # retrieves state info

    def startWork(self):
        self.timer.setInterval(100) # 100 milliseconds
        self.connect(self.timer, SIGNAL('timeout()'), self.iterate)
        self.timer.start()

    def iterate(self):
        # update plane data +++++++++++++++++CHANGE/REMOVE CONVERSION EVENTUALLY+++++++++++
        lat = self.location.latitude(Marble.GeoDataCoordinates.Degree) + self.state.de/111111.0;
        lon = self.location.longitude(Marble.GeoDataCoordinates.Degree) + self.state.dn/111111.0;

        coord = Marble.GeoDataCoordinates(lon, lat, 0.0, Marble.GeoDataCoordinates.Degree);
        self.emit(SIGNAL("coordinatesChanged(PyQt_PyObject)"), coord)

    def finishWork(self):
        self.timer.stop()

#def createStylePlane(style):
#    iconStyle = Marble.GeoDataIconStyle();
#    iconStyle.setIconPath(os.path.join(PWD, 'resources', 'airplane.png'))
#    style.setIconStyle(iconStyle)

class MapWindow(QWidget):
    def __init__(self, uifname = 'map_widget.ui'):
        super(MapWindow, self).__init__()
        button_icon_file = os.path.join(PWD, 'resources', 'airplane.png')
        ui_file = os.path.join(PWD, 'resources', uifname)
        loadUi(ui_file, self, {'MarbleMap' : MarbleMap})
        # there is now a self._marble_map, and there can only be one
        self.setObjectName(uifname)

        map_coords = map_info_parser.get_gps_dict()
        self._home_opts.clear()
        self._home_opts.addItems(list(map_coords))
        self._home_opts.setCurrentIndex(list(map_coords).index(map_info_parser.get_default()))
        self._home_opts.currentIndexChanged[str].connect(self._update_home)

        plane_icon = QPixmap(button_icon_file);
        self._plot_plane_button.setIcon(QIcon(plane_icon))
        self._plot_plane_button.clicked.connect(self._start_plane)
        #self._start_plane() #-------------

        self.plane = Marble.GeoDataPlacemark("Plane")
        self.document = Marble.GeoDataDocument()
        self.document.append(self.plane)
        self._marble_map.model().treeModel().addDocument(self.document)

    def _start_plane(self):
        location = self._marble_map.get_home()
        self.threadFirst = QThread()
        self.firstWorker = Plane(location)
        self.firstWorker.moveToThread(self.threadFirst)

        self.connect(self.firstWorker, SIGNAL("coordinatesChanged(PyQt_PyObject)"),
                self.setPlaneCoordinates, Qt.BlockingQueuedConnection)
        self.connect(self.threadFirst, SIGNAL("started()"), self.firstWorker.startWork)
        self.connect(self.threadFirst, SIGNAL("finished()"), self.firstWorker.finishWork)

        self.threadFirst.start()

    def setPlaneCoordinates(self, coord):
        self.plane.setCoordinate(coord)
        self._marble_map.model().treeModel().updateFeature(self.plane)

    def _update_home(self):
        self._marble_map.change_home(self._home_opts.currentText())

    def close(self):
        super(MapWindow, self).close()
    def save_settings(self, plugin_settings, instance_settings):
        pass
    def restore_settings(self, plugin_settings, instance_settings):
        pass
