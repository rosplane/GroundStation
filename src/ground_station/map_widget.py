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
from fcu_common.msg import GPS

PWD = os.path.dirname(os.path.abspath(__file__))

class StateSubscriber():
    def __init__(self):
        # subscribing to fcu_common/GPS to get plane latitude and longitude
        self.lat = 0.0 # in degrees
        self.lon = 0.0
        rospy.Subscriber("/gps/data", GPS, self.callback)

    def callback(self, GPS):
        self.lat = GPS.latitude
        self.lon = -1.0*GPS.longitude

class Plane(QObject):
    def __init__(self, location):
        QObject.__init__(self)
        self.location = location # if no ros data is received, plane placed at the home_pt
        self.timer = QTimer(self)
        self.state = StateSubscriber() # retrieves state info

    def startWork(self):
        self.timer.setInterval(100) # 100 milliseconds
        self.connect(self.timer, SIGNAL('timeout()'), self.iterate)
        self.timer.start()

    def iterate(self):
        # update plane data
        lat = self.state.lat;
        lon = self.state.lon;
        coord = Marble.GeoDataCoordinates(lon, lat, 0.0, Marble.GeoDataCoordinates.Degree);
        # Advertise coordinate change through PyQt backend
        self.emit(SIGNAL("coordinatesChanged(PyQt_PyObject)"), coord)

    def finishWork(self):
        self.timer.stop()

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

        #self.connect(self.plane, SIGNAL("destroyed()"), self.handleNewPlane)

        self.threadFirst.start()

    def handleNewPlane(self):
        print("plane deleted, yo.")

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
