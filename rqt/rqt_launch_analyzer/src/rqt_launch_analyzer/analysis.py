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
    _ATTRIBUTE_CRE = re.compile(r'^((%s)*|\$\(eval\s+.*\))$' % _SUBST_ARG_RE)

    def __init__(self):
        self._rospack = rospkg.RosPack()

    def parse(self, launch_file_path, external_args, external_envs):
        return self._parse_launch_file(launch_file_path, None, external_args, external_envs)

    def get_package_path(self, package_name):
        try:
            return self._rospack.get_path(package_name)
        except ResourceNotFound:
            return None

    def _parse_launch_file(self, path, parent, external_args, external_envs):
        e = RootLaunchElement(parent, external_args, external_envs)
        e.exists = False
        if path:
            try:
                root_elem = ElementTree.parse(path).getroot()
            except IOError:
                return e
            e.exists = True
            if root_elem.tag == 'launch':
                self._parse_element(e, root_elem)
        return e

    def _parse_element(self, e, xml_elem):
        e.tag = xml_elem.tag
        for name, value in xml_elem.items():
            a = LaunchAttribute(name, value)
            e.attributes[name] = a
            self._evaluate_attribute(a, e.parent)
            if a.name == 'if':
                e.enabled = self._parse_bool(a.evaluated_value)
            elif a.name == 'unless':
                e.enabled = self._parse_bool(a.evaluated_value)
                if e.enabled is not None:
                    e.enabled = not e.enabled

        if e.enabled:
            if e.tag == 'arg' and 'name' in e.attributes:
                name = e.attributes['name'].evaluated_value
                if name is not None:
                    if 'value' in e.attributes:
                        e.parent.set_arg(name, e.attributes['value'].evaluated_value)
                    else:
                        default_value = e.attributes['default'].evaluated_value if 'default' in e.attributes else None
                        value = e.parent.declare_arg(name, default_value)
                        e.exists = value is not None
            elif e.tag == 'env' and 'name' in e.attributes and 'value' in e.attributes:
                name = e.attributes['name'].evaluated_value
                value = e.attributes['value'].evaluated_value
                if name is not None and value is not None:
                    e.parent.set_env(name, value)

            for xml_child_elem in xml_elem:
                child = LaunchElement(e)
                self._parse_element(child, xml_child_elem)
                e.children.append(child)

            if e.tag == 'include' and 'file' in e.attributes:
                file = e.attributes['file'].evaluated_value
                if file is not None:
                    child = self._parse_launch_file(file, e, {}, {})
                    e.children.append(child)

    @staticmethod
    def _parse_bool(value):
        if value is not None:
            value = value.upper()
            if value in ('TRUE', '1'):
                return True
            elif value in ('FALSE', '0'):
                return False
        return None

    def _evaluate_attribute(self, attribute, parent):
        if LaunchFileParser._ATTRIBUTE_CRE.match(attribute.raw_value) is None:
            rospy.logwarn('Attribute "%s" does not match RE' % attribute.raw_value)
        elif attribute.raw_value.startswith('$(eval'):
            sa = self._evaluate_expression(attribute.raw_value, parent)
            attribute.parts.append(sa)
            attribute.evaluated_value = sa.evaluated_value
        else:
            attribute.evaluated_value = ''
            for match in LaunchFileParser._SUBST_ARG_CRE.finditer(attribute.raw_value):
                sa = self._evaluate_match(match, parent)
                attribute.parts.append(sa)
                if sa.evaluated_value is None:
                    attribute.evaluated_value = None
                elif attribute.evaluated_value is not None:
                    attribute.evaluated_value += sa.evaluated_value

    def _evaluate_match(self, match, parent):
        if match.group(1):  # plain text
            sa = SubstitutionArg(None, None, match.group(0), match.group(0))
        else:
            sa = SubstitutionArg(match.group(2), match.group(3), match.group(0), None)
            if sa.tag == 'env':
                sa.evaluated_value = parent.get_env(sa.key)
            elif sa.tag == 'optenv':
                value = parent.get_env(sa.key)
                sa.evaluated_value = value if value is not None else match.group(4)
            elif sa.tag == 'anon':
                sa.evaluated_value = parent.get_anon(sa.key)
            elif sa.tag == 'arg':
                sa.evaluated_value = parent.get_arg(sa.key)
                if sa.evaluated_value is None:
                    sa.error_message = 'No arg named "%s"' % sa.key
            elif sa.tag == 'find':
                try:
                    sa.evaluated_value = self.get_package_path(sa.key)
                except ResourceNotFound as e:
                    sa.error_message = str(e)
        return sa

    @staticmethod
    def _evaluate_expression(raw_value, parent):
        expression = raw_value[7:-1]
        sa = SubstitutionArg('eval', expression, raw_value, None)
        try:
            sa.evaluated_value = parent.eval_expression(expression)
        except Exception as e:
            sa.error_message = str(e)
        return sa


class LaunchElement(object):
    def __init__(self, parent=None):
        self.parent = parent
        self.children = []
        self.tag = None
        self.attributes = {}
        self.enabled = True
        self.exists = None
        self._envs = {}
        self._anons = {}
        self._args = {}

    def get_env(self, key):
        if key in self._envs:
            return self._envs[key]
        return self.parent.get_env(key)

    def set_env(self, key, value):
        self._envs[key] = value

    def get_anon(self, key):
        if key in self._anons:
            return self._anons[key]
        return self.parent.get_anon(key)

    def get_arg(self, key):
        if key in self._args:
            return self._args[key]
        return self.parent.get_arg(key)

    def get_args(self):
        args = dict(self.parent.get_args())
        args.update(self._args)
        return args

    def set_arg(self, key, value):
        self._args[key] = value

    def get_passed_arg(self, key):
        if key in self._args:
            return self._args[key]
        return self.parent.get_passed_arg(key)

    def declare_arg(self, key, default_value):
        if key not in self._args:
            value = self.get_passed_arg(key)
            self._args[key] = value if value is not None else default_value
        return self._args[key]

    def eval_expression(self, expression):
        eval_globals = {'arg': self.get_arg, 'env': self.get_env}
        eval_globals.update(self.get_args())
        return str(eval(expression, eval_globals, {}))


class RootLaunchElement(LaunchElement):
    """LaunchElement corresponding to the <launch> tag"""

    def __init__(self, parent, external_args, external_envs):
        super(RootLaunchElement, self).__init__(parent)
        self.external_args = external_args
        self.external_envs = external_envs
        self.required_args = []
        self.required_envs = []

    def get_env(self, key):
        if key in self._envs:
            return self._envs[key]
        elif self.parent:
            return self.parent.get_env(key)
        self.required_envs.append(key)
        if key in self.external_envs:
            return self.external_envs[key]
        return os.environ.get(key)

    def get_anon(self, key):
        if key not in self._anons:
            self._anons[key] = '%s-%d' % (key, random.randrange(1000))
        return self._anons[key]

    def get_arg(self, key):
        if key in self._args:
            return self._args[key]
        return None

    def get_args(self):
        return dict(self._args)

    def get_passed_arg(self, key):
        if key in self._args:
            return self._args[key]
        elif self.parent:
            # Look for args inside <include> tag:
            if key in self.parent._args:
                return self.parent._args[key]
        self.required_args.append(key)
        if key in self.external_args:
            return self.external_args[key]
        return None


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
        self.error_message = None
