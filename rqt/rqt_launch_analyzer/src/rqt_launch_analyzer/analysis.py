import os
import re
import random
import rospy
import rospkg
from rospkg.common import ResourceNotFound
from xml.etree import ElementTree

class LaunchFileParser(object):
    _SUBST_ARG_RE = r'([^$]+)|\$\(([a-z]+)\s+([^\s\)]+)\s*(\w+)?\)'
    _SUBST_ARG_CRE = re.compile(_SUBST_ARG_RE)
    _ATTRIBUTE_CRE = re.compile('^(%s)*$' % _SUBST_ARG_RE)

    def __init__(self):
        self._rospack = rospkg.RosPack()

    def parse(self, package_name, launch_file_name):
        launch_file_path = os.path.join(self._rospack.get_path(package_name), 'launch', launch_file_name)
        root_sc = SubstitutionContext()
        return (self._parse_launch_file(launch_file_path, None, root_sc), root_sc)

    def _parse_launch_file(self, path, parent, sc):
        try:
            root_elem = ElementTree.parse(path).getroot()
        except Exception, e:
            rospy.logwarn('Launch file %s cannot be read (%s: %s)' % (
                    path, type(e).__name__, e.message))
            return
        if root_elem.tag != 'launch':
            rospy.logwarn('Launch file %s: root XML node is not <launch>' % path)
            return
        return self._parse_element(root_elem, parent, sc)

    def _parse_element(self, xml_elem, parent, sc):
        e = LaunchElement(parent, xml_elem.tag)
        for name, value in xml_elem.items():
            a = LaunchAttribute(name, value)
            e.attributes[name] = a
            self._evaluate_attribute(a, sc)
            if a.name in ('if', 'unless'):
                if a.evaluated_value in ('true', 1):
                    e.enabled = a.name == 'if'
                elif a.evaluated_value in ('false', 0):
                    e.enabled = a.name == 'unless'
                else:
                    e.enabled = None # invalid

        if e.enabled:
            if e.tag == 'arg' and 'name' in e.attributes:
                name = e.attributes['name'].evaluated_value
                if name is not None:
                    if 'value' in e.attributes:
                        sc.set_arg(name, e.attributes['value'].evaluated_value)
                    elif sc.get_arg(name) is None and 'default' in e.attributes:
                        sc.set_arg(name, e.attributes['default'].evaluated_value)
            elif e.tag == 'env' and 'name' in e.attributes and 'value' in e.attributes:
                name = e.attributes['name'].evaluated_value
                value = e.attributes['value'].evaluated_value
                if name is not None and value is not None:
                    sc.set_env(name, value)

            child_sc = SubstitutionContext(sc)
            for xml_child_elem in xml_elem:
                child = self._parse_element(xml_child_elem, e, child_sc)
                e.children.append(child)

            if e.tag == 'include' and 'file' in e.attributes:
                file = e.attributes['file'].evaluated_value
                if file is not None:
                    child = self._parse_launch_file(file, e, child_sc)
                    e.children.append(child)
        return e

    def _evaluate_attribute(self, attribute, sc):
        if LaunchFileParser._ATTRIBUTE_CRE.match(attribute.raw_value) is None:
            rospy.logwarn('Attribute %s does not match RE' % attribute.raw_value)
        else:
            attribute.evaluated_value = ''
            for match in LaunchFileParser._SUBST_ARG_CRE.finditer(attribute.raw_value):
                sa = self._evaluate_match(match, sc)
                attribute.parts.append(sa)
                if sa.evaluated_value is None:
                    attribute.evaluated_value = None
                elif attribute.evaluated_value is not None:
                    attribute.evaluated_value += sa.evaluated_value

    def _evaluate_match(self, match, sc):
        if match.group(1): # plain text
            sa = SubstitutionArg(None, None, match.group(0), match.group(0))
        else:
            sa = SubstitutionArg(match.group(2), match.group(3), match.group(0), None)
            if sa.tag == 'env':
                sa.evaluated_value = sc.get_env(sa.key)
            elif sa.tag == 'optenv':
                value = sc.get_env(sa.key)
                sa.evaluated_value = value if value is not None else match.group(4)
            elif sa.tag == 'anon':
                sa.evaluated_value = sc.get_anon(sa.key)
            elif sa.tag == 'arg':
                sa.evaluated_value = sc.get_arg(match.group(3))
            elif sa.tag == 'find':
                try:
                    sa.evaluated_value = self._rospack.get_path(sa.key)
                except ResourceNotFound:
                    pass
        return sa


class LaunchElement(object):
    def __init__(self, parent=None, tag=None):
        self.parent = parent
        self.children = []
        self.tag = tag
        self.enabled = True
        self.attributes = {}
        self.exists = None


class LaunchAttribute(object):
    def __init__(self, name, raw_value):
        self.name = name
        self.raw_value = raw_value
        self.evaluated_value = None
        self.parts = []


class SubstitutionArg(object):
    def __init__(self, tag, key, raw_value, evaluated_value):
        self.tag = tag
        self.key = key
        self.raw_value = raw_value
        self.evaluated_value = evaluated_value


class SubstitutionContext(object):
    def __init__(self, parent=None):
        self._env = {}
        self._anon = {}
        self._arg = {}
        self.parent = parent

    def get_env(self, key):
        if key in self._env:
            value = self._env[key]
            value[1] += 1
            return value[0]
        elif self.parent:
            return self.parent.get_env(key)
        else:
            self._env[key] = [None, 1]
            return None

    def set_env(self, key, value):
        if key in self._env:
            self._env[key][0] = value
        else:
            self._env[key] = [value, 0]

    def get_anon(self, key):
        if key in self._anon:
            value = self._anon[key]
            value[1] += 1
            return value[0]
        else:
            result = '%s-%d' % (key, random.randrange(1000))
            self._anon[key] = [result, 1]
            return result

    def get_arg(self, key):
        if key in self._arg:
            value = self._arg[key]
        elif self.parent and key in self.parent._arg:
            value = self.parent._arg[key]
        else:
            value = [None, 0]
            self._arg[key] = value
        value[1] += 1
        return value[0]

    def set_arg(self, key, value):
        if key in self._arg:
            self._arg[key][0] = value
        else:
            self._arg[key] = [value, 0]




