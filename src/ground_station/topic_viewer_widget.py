from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt #
from python_qt_binding.QtGui import QIcon #

from python_qt_binding.QtWidgets import (QAction, QMenu, QMessageBox,
                                         QTreeView, QWidget)
import roslib, rosmsg, rospkg, rospy, os

from .messages_tree_view import MessagesTreeView
from rqt_py_common import rosaction
#from rqt_py_common import rqt_plot #??????????????
from rqt_console.text_browse_dialog import TextBrowseDialog

PWD = os.path.dirname(os.path.abspath(__file__))

'''
PENDING:
    - find out which specific topics & sub-topics are wanted
    - any non-graphs wanted?
    - other functionality as well?
    - make two drop-down menus to get to the individual, plottable items
    - do to rqt_plot what rqt_msg did to MessagesTreeView...
'''

class TopicViewer(QWidget):
    def __init__(self, uifname = 'topic_viewer.ui'):
        super(TopicViewer, self).__init__()
        #self._rospack = rospkg.RosPack() #?
        ui_file = os.path.join(PWD, 'resources', uifname)
        loadUi(ui_file, self, {'MessagesTreeView' : MessagesTreeView})
        #loadUi(ui_file, self) # third arg is custom widget
        self.setObjectName(uifname)
        #self._mode = mode # (=rosmsg.MODE_MSG)

        self._msgs.clear()
        self._msgs.addItems(rosmsg.list_msgs('fcu_common')) # get list of fcu_common msgs
        self._draw_graph()
        self._msgs.currentIndexChanged[str].connect(self._draw_graph)
        #====
        '''
        packages = sorted([pkg_tuple[0] for pkg_tuple in
                           rosmsg.iterate_packages(self._rospack, self._mode)])
        print(rospy.logdebug('pkgs={}'.format(packages)))
    def _refresh_msgs(self, package):
        self._msgs = rosmsg.list_msgs(package)
        #
        '''
    def _draw_graph(self):
        print(self._msgs.currentText()) #----
    def close(self):
        pass
    def save_settings(self, plugin_settings, instance_settings):
        pass
    def restore_settings(self, plugin_settings, instance_settings):
        pass
