import os
import re
import random
import rospy
import rospkg
import xml.etree.ElementTree as ElementTree

class LaunchFileParser(object):
    _SUBST_ARG_RE = r'([^$]+)|\$\(([a-z]+)(\s+\w+)(\s+\w+)?\)'
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
            rospy.warn('Launch file %s cannot be read (%s: %s)' % (
                    path, type(e).__name__, e.message))
            return
        if root_elem.tag != 'launch':
            rospy.warn('Launch file %s: root XML node is not <launch>' % path)
            return
        return self._parse_element(root_elem, parent, sc)

    def _parse_element(self, xml_elem, parent, sc):
        e = LaunchElement(parent, xml_elem.tag)
        for name, value in xml_elem.items():
            a = LaunchAttribute(name, value)
            self._evaluate(a, sc)
            e.attributes.append(a)
        child_sc = SubstitutionContext(sc)
        for xml_child_elem in xml_elem:
            child = self._parse_element(xml_child_elem, e, child_sc)
            e.children.append(child)
        return e

    def _evaluate(self, attribute, sc):
        if LaunchFileParser._ATTRIBUTE_CRE.match(attribute.raw_value) is None:
            rospy.warn('Attribute %s does not match RE' % attribute.raw_value)
        else:
            evaluated_value = ''
            for match in LaunchFileParser._SUBST_ARG_CRE.finditer(attribute.raw_value):
                evaluated_value += '<%s>' % match.group(0)
            attribute.evaluated_value = evaluated_value


class LaunchElement(object):
    def __init__(self, parent=None, tag=None):
        self.parent = parent
        self.children = []
        self.tag = tag
        self.enabled = None
        self.attributes = []
        self.exists = None


class LaunchAttribute(object):
    def __init__(self, name, raw_value):
        self.name = name
        self.raw_value = raw_value
        self.evaluated_value = None
        self.parts = []


class SubstitutionArg(object):
    def __init__(self, tag, key, raw_value, default_value=None):
        self.tag = tag
        self.key = key
        self.raw_value = raw_value
        self.evaluated_value = None
        self.default_value = default_value


class SubstitutionContext(object):
    def __init__(self, parent=None):
        self._env = {}
        self._anon = {}
        self._arg = {}
        self.parent = parent
        
    def get_env(key):
        if key in self._env:
            value = self._env[key]
            value[1] += 1
            return value[0]
        elif self.parent:
            return self.parent.get_env(key)
        else:
            self._env[key] = [None, 1]
            return None

    def get_anon(key):
        if key in self._anon:
            value = self._anon[key]
            value[1] += 1
            return value[0]
        else:
            result = '%s-%d' % (key, random.randrange(1000))
            self._anon[key] = [result, 1]
            return result

    def get_arg(key):
        if key in self._arg:
            value = self._arg[key]
            value[1] += 1
            return value[0]
        else:
            self._arg[key] = [None, 1]
            return None






