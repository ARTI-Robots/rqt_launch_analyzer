import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import (QAbstractItemModel, QModelIndex, Qt)
from python_qt_binding.QtGui import (QWidget, QIcon, QFont, QBrush, QColor)

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
        self._widget.actions_label.linkHovered.connect(self._action_link_hovered)

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
        # Connect selection model signal. The selection model is recreated on tree.setModel(), so we have to do that
        # here:
        self._widget.launch_file_tree_view.selectionModel().currentChanged.connect(self._current_tree_item_changed)

    def _current_tree_item_changed(self, current, previous):
        if current.isValid():
            item = current.internalPointer()
            self._widget.xml_label.setText(item.get_xml())
            actions = []
            if item.elem.tag == 'include' and 'file' in item.elem.attributes:
                a = item.elem.attributes['file']
                for p in a.parts:
                    if p.tag == 'find' and p.evaluated_value is not None:
                        actions.append('<a href="%s">Package</a>' % p.evaluated_value)
                        break
                if a.evaluated_value is not None:
                    actions.append('<a href="%s">File</a>' % a.evaluated_value)
            self._widget.actions_label.setText(' '.join(actions))

    def _action_link_hovered(self, link):
        self._widget.actions_label.setToolTip(link)

    def _dump_elem(self, elem, indentation):
        attrs = ''
        for attr in elem.attributes:
            attrs += ' %s="%s"' % (attr.name, attr.evaluated_value)
        print '%s<%s%s>' % (indentation, elem.tag, attrs)
        for child in elem.children:
            self._dump_elem(child, indentation + '  ')


class LaunchTreeModel(QAbstractItemModel):
    _icon_cache = {}

    def __init__(self, root_elem):
        super(QAbstractItemModel, self).__init__()
        self.root_elem = root_elem
        self._build()

    def index(self, row, column, parent):
        p = parent.internalPointer() if parent.isValid() else self._root_item
        if 0 <= row < len(p.children) and column == 0:
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
            elif role == Qt.ToolTipRole:
                return item.get_tool_tip()
            elif role == Qt.FontRole:
                return item.get_font()
            elif role == Qt.ForegroundRole:
                return item.get_foreground()
        return None

    def _build(self):
        self._root_item = LaunchTreeItem(None, None, None)
        self._root_item._add_child(self.root_elem)


class LaunchTreeItemState:
    OK = 0
    ERROR = 1
    CHILD_ERROR = 2
    DISABLED = 3


def _make_disabled_font():
    font = QFont()
    font.setStrikeOut(True)
    return font


class LaunchTreeItem(object):
    _ICONS = {
        'DEFAULT': 'help-contents',
        'launch': 'media-playback-start',
        'node': 'application-x-executable',
        'machine': 'computer',
        'include': 'edit-copy',
        'remap': 'document-properties',
        'env': 'document-properties',
        'param': 'document-properties',
        'rosparam': 'document-properties',
        'group': 'folder',
        'test': 'application-x-executable',
        'arg': 'insert-text'
    }

    _FONTS = {
        LaunchTreeItemState.OK: None,
        LaunchTreeItemState.ERROR: None,
        LaunchTreeItemState.CHILD_ERROR: None,
        LaunchTreeItemState.DISABLED: None #_make_disabled_font()
    }

    _FOREGROUNDS = {
        LaunchTreeItemState.OK: QColor(Qt.black),
        LaunchTreeItemState.ERROR: QColor(Qt.red),
        LaunchTreeItemState.CHILD_ERROR: QColor(Qt.darkRed),
        LaunchTreeItemState.DISABLED: QColor(Qt.darkGray)
    }

    def __init__(self, parent, row, elem):
        self.parent = parent
        self.row = row
        self.elem = elem
        self.children = []

        if elem is not None:
            self.icon = LaunchTreeItem._ICONS[elem.tag if elem.tag in LaunchTreeItem._ICONS else 'DEFAULT']
            self.state = LaunchTreeItemState.OK if elem.enabled else LaunchTreeItemState.DISABLED
            for a in elem.attributes.itervalues():
                if a.evaluated_value is None:
                    self.state = LaunchTreeItemState.ERROR
            self._init_title()
        else:
            self.icon = LaunchTreeItem._ICONS['DEFAULT']
            self.state = LaunchTreeItemState.OK
            self.title = ''

    def get_xml(self):
        # Rebuild XML tag:
        xml = '<%s' % self.elem.tag
        for a in self.elem.attributes.itervalues():
            xml += ' %s="%s"' % (a.name, a.raw_value)
        return "%s>" % xml

    def get_tool_tip(self):
        return None

    def get_font(self):
        return LaunchTreeItem._FONTS[self.state]

    def get_foreground(self):
        return LaunchTreeItem._FOREGROUNDS[self.state]

    def _add_child(self, elem):
        child = LaunchTreeItem(self, len(self.children), elem)
        self.children.append(child)
        for child_elem in elem.children:
            child._add_child(child_elem)
        if self.state in (LaunchTreeItemState.OK, LaunchTreeItemState.DISABLED) and child.state in (
                LaunchTreeItemState.ERROR, LaunchTreeItemState.CHILD_ERROR):
            self.state = LaunchTreeItemState.CHILD_ERROR

    def _init_title(self):
        self.title = '<%s>' % self.elem.tag
        if self.elem.tag == 'node':
            self.title += ' %s (pkg:%s/%s)' % (self._format_attribute('name'), self._format_attribute('pkg'),
                                               self._format_attribute('type'))
        elif self.elem.tag == 'include':
            self.title += ' %s' % (self._format_attribute('file'))
        elif self.elem.tag == 'arg':
            self.title += ' %s' % self._format_attribute('name')
            if 'value' in self.elem.attributes:
                self.title += ' = %s' % self._format_attribute('value')
            if 'default' in self.elem.attributes:
                self.title += ' (default: %s)' % self._format_attribute('default')
        elif self.elem.tag == 'param':
            self.title += ' %s = %s' % (self._format_attribute('name'), self._format_attribute('value'))
        elif self.elem.tag == 'remap':
            self.title += u' %s \u2192 %s' % (self._format_attribute('from'), self._format_attribute('to'))

    def _format_attribute(self, name):
        if name in self.elem.attributes:
            a = self.elem.attributes[name]
            result = ''
            for p in a.parts:
                if p.tag == 'find':
                    result += 'pkg:%s' % p.key
                elif p.evaluated_value is not None:
                    result += p.evaluated_value
                else:
                    result += p.raw_value
            return result
        else:
            return '???'
