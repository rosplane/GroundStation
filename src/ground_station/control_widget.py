from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

import os

PWD = os.path.dirname(os.path.abspath(__file__))

class ControlWindow(QWidget):
    def __init__(self, uifname = 'control.ui'):
        super(ControlWindow, self).__init__()
        ui_file = os.path.join(PWD, 'resources', uifname)
        loadUi(ui_file, self)
        self.setObjectName(uifname)

    def close(self):
        super(NavView, self).close()
    def save_settings(self, plugin_settings, instance_settings):
        pass
    def restore_settings(self, plugin_settings, instance_settings):
        pass
