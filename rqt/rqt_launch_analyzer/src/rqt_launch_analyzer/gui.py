import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import (QAbstractItemModel, QModelIndex, QUrl, Qt)
from python_qt_binding.QtGui import (QDesktopServices, QWidget, QIcon, QFont, QBrush, QColor)

from .analysis import LaunchFileParser


PACKAGE_NAME = 'rqt_launch_analyzer'


class Gui(Plugin):
    def __init__(self, context):
        super(Gui, self).__init__(context)
        self.setObjectName(PACKAGE_NAME)

        self._parser = LaunchFileParser()
        self._current_package_name = None
        self._current_launch_file_name = None

        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path(PACKAGE_NAME), 'resource', 'launch_analyzer.ui')
        loadUi(ui_file, self._widget)

        self._widget.refresh_button.setIcon(QIcon.fromTheme('view-refresh'))
        self._widget.refresh_button.pressed.connect(self._refresh)
        self._widget.xml_label.linkHovered.connect(lambda link: self._label_link_hovered(self._widget.xml_label, link))
        self._widget.xml_label.linkActivated.connect(self._label_link_activated)
        self._widget.current_file_label.linkHovered.connect(
            lambda link: self._label_link_hovered(self._widget.current_file_label, link))

        context.add_widget(self._widget)

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('package_edit_text', self._widget.package_edit.text())
        instance_settings.set_value('launch_file_edit_text', self._widget.launch_file_edit.text())

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.package_edit.setText(instance_settings.value('package_edit_text', ''))
        self._widget.launch_file_edit.setText(instance_settings.value('launch_file_edit_text', ''))

    def _refresh(self):
        root_elem = self._parser.parse(self._widget.package_edit.text(), self._widget.launch_file_edit.text(), {}, {})
        self._widget.launch_file_tree_view.setModel(LaunchTreeModel(root_elem))
        self._current_package_name = self._widget.package_edit.text()
        self._current_launch_file_name = self._widget.launch_file_edit.text()
        # Connect selection model signal. The selection model is recreated on tree.setModel(), so we have to do that
        # here:
        self._widget.launch_file_tree_view.selectionModel().currentChanged.connect(self._current_tree_item_changed)

    def _current_tree_item_changed(self, current, previous):
        item = current.internalPointer() if current.isValid() else None
        self._update_xml_label(item)
        self._update_current_file_label(item)

    def _update_xml_label(self, item):
        if item is None:
            self._widget.xml_label.setText('')
            return

        # Rebuild XML tag, inserting links for interesting parts:
        xml = '&lt;%s' % item.elem.tag
        for a in item.elem.attributes.itervalues():
            if a.name == 'pkg': # simple heuristics
                path = self._parser.get_package_path(a.evaluated_value) if a.evaluated_value is not None else None
                xml += ' ' + self._make_link(a.name, path, True)
            elif a.name == 'file': # simple heuristics
                xml += ' ' + self._make_link(a.name, a.evaluated_value, True)
            else:
                xml += ' ' + Qt.escape(a.name)

            xml += '="'
            for p in a.parts:
                if p.tag is not None:
                    xml += self._make_link(p.raw_value, p.evaluated_value, p.tag == 'find')
                else:
                    xml += Qt.escape(p.raw_value)
            xml += '"'
        self._widget.xml_label.setText("%s&gt;" % xml)

    def _make_link(self, raw_value, evaluated_value, is_path):
        if evaluated_value is None:
            return '<a href="#" style="color:red;text-decoration:none">%s</a>' % Qt.escape(raw_value)
        elif is_path:
            return '<a href="%s">%s</a>' % (Qt.escape(evaluated_value), Qt.escape(raw_value))
        else:
            return '<a href="#%s" style="color:darkgreen;text-decoration:none">%s</a>' % (Qt.escape(evaluated_value), Qt.escape(raw_value))

    def _update_current_file_label(self, item):
        if item is None:
            self._widget.current_file_label.setText('')
            return

        package_name = None
        launch_file_name = None
        # Find containing <launch>:
        while item.elem is not None and item.elem.tag != 'launch':
            item = item.parent
        # Find containing <include>:
        while item.elem is not None and item.elem.tag != 'include':
            item = item.parent
        if item.elem is None: # Topmost <launch>
            package_name = self._current_package_name
            launch_file_name = '/launch/' + self._current_launch_file_name
        elif 'file' in item.elem.attributes:
            a = item.elem.attributes['file']
            if len(a.parts) > 0 and a.parts[0].tag == 'find': # Normal file parameter structure, woo-hoo
                package_name = a.parts[0].key
                launch_file_name = ''
                for j in xrange(1, len(a.parts)):
                    launch_file_name += a.parts[j].evaluated_value

        package_path = self._parser.get_package_path(package_name)
        if package_path is not None and launch_file_name is not None:
            self._widget.current_file_label.setText(
                'Current launch file: <a href="%s">%s</a> in package <a href="%s">%s</a>' % (
                    package_path + launch_file_name, launch_file_name, package_path, package_name))
        else:
            self._widget.current_file_label.setText('')

    def _label_link_hovered(self, label, link):
        if link.startswith('#'):
            link = link[1:]
            label.unsetCursor()
        label.setToolTip(link)

    def _label_link_activated(self, link):
        if not link.startswith('#'):
            QDesktopServices.openUrl(QUrl(link))


class LaunchTreeModel(QAbstractItemModel):
    _icon_cache = {}

    def __init__(self, root_elem):
        super(LaunchTreeModel, self).__init__()
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
            if elem.exists is False:
                self.state = LaunchTreeItemState.ERROR
            else:
                for a in elem.attributes.itervalues():
                    if a.evaluated_value is None:
                        self.state = LaunchTreeItemState.ERROR
            self._init_title()
        else:
            self.icon = LaunchTreeItem._ICONS['DEFAULT']
            self.state = LaunchTreeItemState.OK
            self.title = ''

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
        elif self.elem.tag == 'rosparam':
            if 'command' in self.elem.attributes:
                self.title += ' %s' % self._format_attribute('command')
            if 'file' in self.elem.attributes:
                self.title += ' %s' % self._format_attribute('file')
        elif self.elem.exists is False:
            self.title += ' [does not exist]'

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
