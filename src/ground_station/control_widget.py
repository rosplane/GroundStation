from python_qt_binding.QtWidgets import QGraphicsView

class ControlWindow(QGraphicsView):
    def __init__(self):
        super(ControlWindow, self).__init__()
    def close(self):
        super(NavView, self).close()
    def save_settings(self, plugin_settings, instance_settings):
        pass
    def restore_settings(self, plugin_settings, instance_settings):
        pass
