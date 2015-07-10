import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
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
        self._dump_elem(root_elem, '')
        
    def _dump_elem(self, elem, indentation):
        attrs = ''
        for attr in elem.attributes:
            attrs += ' %s="%s"' % (attr.name, attr.evaluated_value)
        print '%s<%s%s>' % (indentation, elem.tag, attrs)
        for child in elem.children:
            self._dump_elem(child, indentation + '  ')


