import argparse
from python_qt_binding import QT_BINDING
from python_qt_binding.QtCore import qDebug
from python_qt_binding.QtWidgets import QWidget, QBoxLayout, QVBoxLayout, QHBoxLayout, QPushButton

# Custom Widgets
from .map_widget import MapWindow
from .plot_widget import PlotWidget
from .data_plot import DataPlot
from .control_widget import ControlWindow
from .artificial_horizon import ArtificialHorizon

class GroundStationWidget(QWidget):

    def __init__(self):
        super(GroundStationWidget, self).__init__()

        # The layout of the ground station window
        self._principle_layout = QBoxLayout(0) # main layout is horizontal (0)
        self._map_layout = QVBoxLayout()
        self._principle_layout.addLayout(self._map_layout, 4)
        self._control_layout = QVBoxLayout()
        self._principle_layout.addLayout(self._control_layout, 3)

        self.setAcceptDrops(False) # Dragging and Dropping not permitted
        self.setWindowTitle('ROS_PLANE Ground Station')

        #=============================
        self._mw = MapWindow()
        self._map_layout.addWidget(self._mw)
        self._tv = PlotWidget()
        self._data_plot = DataPlot(self._tv)
        self._data_plot.set_autoscale(x=False)
        self._data_plot.set_autoscale(y=DataPlot.SCALE_EXTEND|DataPlot.SCALE_VISIBLE)
        self._data_plot.set_xlim([0, 10.0])
        self._tv.switch_data_plot_widget(self._data_plot)

        self._control_layout.addWidget(self._tv, 1) # ratio of these numbers determines window proportions
        self._ah = ArtificialHorizon()
        self._control_layout.addWidget(self._ah, 1)
        #=============================
        print('fake init')

        self.setLayout(self._principle_layout)

    def save_settings(self, plugin_settings, instance_settings): # have a file to read and write from
        print('fake save') # < prints to terminal

    def restore_settings(self, plugin_settings, instance_settings):
        print('fake restore')
