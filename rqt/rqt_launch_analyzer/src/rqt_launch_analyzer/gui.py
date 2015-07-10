import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import (QAbstractItemModel, QModelIndex, Qt)
from python_qt_binding.QtGui import (QWidget, QIcon)

from .analysis import LaunchFileParser


PACKAGE_NAME = 'rqt_launch_analyzer'


class Gui(Plugin):
    def __init__(self, context):
        super(Gui, self).__init__(context)
        self.setObjectName(PACKAGE_NAME)

        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path(PACKAGE_NAME), 'resource', 'launch_analyzer.ui')
        loadUi(ui_file, self._widget)

        self._widget.refresh_button.setIcon(QIcon.fromTheme('view-refresh'))
        self._widget.refresh_button.pressed.connect(self._refresh)
        context.add_widget(self._widget)

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('package_edit_text', self._widget.package_edit.text())
        instance_settings.set_value('launch_file_edit_text', self._widget.launch_file_edit.text())

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.package_edit.setText(instance_settings.value('package_edit_text', ''))
        self._widget.launch_file_edit.setText(instance_settings.value('launch_file_edit_text', ''))

    def _refresh(self):
        parser = LaunchFileParser()
        root_elem, sc = parser.parse(self._widget.package_edit.text(), self._widget.launch_file_edit.text())
        #self._dump_elem(root_elem, '')
        self._widget.launch_file_tree_view.setModel(LaunchTreeModel(root_elem))

    def _dump_elem(self, elem, indentation):
        attrs = ''
        for attr in elem.attributes:
            attrs += ' %s="%s"' % (attr.name, attr.evaluated_value)
        print '%s<%s%s>' % (indentation, elem.tag, attrs)
        for child in elem.children:
            self._dump_elem(child, indentation + '  ')


class LaunchTreeItemState:
    OK = 0
    ERROR = 1
    CHILD_ERROR = 2
    DISABLED = 3


class LaunchTreeModel(QAbstractItemModel):
    _ICONS = {
        'DEFAULT': 'help-contents',
        'launch': 'media-playback-start',
        'node': 'application-x-executable',
        'machine': 'computer',
        'include': 'media-playback-start',
        'remap': 'document-properties',
        'env': 'document-properties',
        'param': 'document-properties',
        'rosparam': 'document-properties',
        'group': 'folder',
        'test': 'application-x-executable',
        'arg': 'insert-text'
    }

    _icon_cache = {}

    def __init__(self, root_elem):
        super(QAbstractItemModel, self).__init__()
        self.root_elem = root_elem
        self._build()

    def index(self, row, column, parent):
        p = parent.internalPointer() if parent.isValid() else self._root_item
        if row >= 0 and row < len(p.children) and column == 0:
            return self.createIndex(row, column, p.children[row])
        return QModelIndex()

    def parent(self, index):
        if index.isValid():
            item = index.internalPointer()
            if item.parent is not self._root_item:
                return self.createIndex(item.parent.row, 0, item.parent)
        return QModelIndex()

    def rowCount(self, parent):
        p = parent.internalPointer() if parent.isValid() else self._root_item
        return len(p.children)

    def columnCount(self, parent):
        return 1

    def data(self, index, role):
        if index.isValid():
            item = index.internalPointer()
            if role == Qt.DisplayRole:
                return item.title
            elif role == Qt.DecorationRole:
                if item.icon not in self._icon_cache:
                    self._icon_cache[item.icon] = QIcon.fromTheme(item.icon)
                return self._icon_cache[item.icon]
        return None

    def _build(self):
        self._root_item = LaunchTreeItem(None, None, None, None, None)
        self._add_child(self._root_item, self.root_elem)

    def _add_child(self, item, elem):
        icon = LaunchTreeModel._ICONS[elem.tag if elem.tag in LaunchTreeModel._ICONS else 'DEFAULT']
        state = LaunchTreeItemState.OK if elem.enabled else LaunchTreeItemState.DISABLED
        for a in elem.attributes:
            if a.evaluated_value is None:
                state = LaunchTreeItemState.ERROR

        child = LaunchTreeItem(item, len(item.children), elem.tag, icon, state)
        item.children.append(child)
        for child_elem in elem.children:
            child_child = self._add_child(child, child_elem)
            if child_child.state in (LaunchTreeItemState.ERROR, LaunchTreeItemState.CHILD_ERROR):
                child.state = LaunchTreeItemState.CHILD_ERROR
        return child

class LaunchTreeItem(object):
    def __init__(self, parent, row, title, icon, state):
        self.parent = parent
        self.row = row
        self.title = title
        self.icon = icon
        self.state = state
        self.children = []
