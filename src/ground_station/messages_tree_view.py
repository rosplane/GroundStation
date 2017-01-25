from rqt_py_common.message_tree_widget import MessageTreeWidget
from rqt_py_common.message_tree_model import MessageTreeModel

class MessagesTreeModel(MessageTreeModel):
    def __init__(self, parent=None):
        super(MessagesTreeModel, self).__init__()
        self.setHorizontalHeaderLabels([self.tr('One'),
                                        self.tr('Two'),
                                        self.tr('Three')])

class MessagesTreeView(MessageTreeWidget):
    def __init__(self, parent=None):
        super(MessagesTreeView, self).__init__()
        self.setModel(MessagesTreeModel(self))

    def _recursive_set_editable(self, item, editable=True):
        item.setEditable(editable)
        for row in range(item.rowCount()):
            for col in range(item.columnCount()):
                self._recursive_set_editable(item.child(row, col), editable)
