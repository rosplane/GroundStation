from python_qt_binding import loadUi
from PyQt4.Qt import *
from PyQt4 import QtGui

try:
    from PyQt4.QtCore import QString
except ImportError:
    QString = type("")

import os
import map_info_parser

PWD = os.path.dirname(os.path.abspath(__file__))

class CmWindow(QWidget):
    def __init__(self, _marble_map, uifname = 'cm_window.ui'):
        super(CmWindow, self).__init__()
        self._marble_map = _marble_map
        ui_file = os.path.join(PWD, 'resources', uifname)
        loadUi(ui_file, self)
        self.setObjectName(uifname)

    def closeEvent(self, QCloseEvent):
        pass
