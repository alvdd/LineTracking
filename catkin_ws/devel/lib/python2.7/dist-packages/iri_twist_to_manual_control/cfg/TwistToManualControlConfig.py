## *********************************************************
##
## File autogenerated for the iri_twist_to_manual_control package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 245, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 290, 'description': 'Minimum output steering value', 'max': 180, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'steering_min', 'edit_method': '', 'default': 45, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 290, 'description': 'Zero output steering value', 'max': 180, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'steering_zero', 'edit_method': '', 'default': 90, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 290, 'description': 'Maximum output steering value', 'max': 180, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'steering_max', 'edit_method': '', 'default': 135, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 290, 'description': 'Maximum input angular speed', 'max': 5.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'angular_sat_min', 'edit_method': '', 'default': -0.5, 'level': 0, 'min': -5.0, 'type': 'double'}, {'srcline': 290, 'description': 'Minimum input angular speed', 'max': 5.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'angular_sat_max', 'edit_method': '', 'default': 0.5, 'level': 0, 'min': -5.0, 'type': 'double'}, {'srcline': 290, 'description': 'Minimum output speed value', 'max': 1000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'speed_min', 'edit_method': '', 'default': -500, 'level': 0, 'min': -1000, 'type': 'int'}, {'srcline': 290, 'description': 'Zero output speed value', 'max': 1000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'speed_zero', 'edit_method': '', 'default': 0, 'level': 0, 'min': -1000, 'type': 'int'}, {'srcline': 290, 'description': 'Maximum output speed value', 'max': 1000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'speed_max', 'edit_method': '', 'default': 500, 'level': 0, 'min': -1000, 'type': 'int'}, {'srcline': 290, 'description': 'Maximum input linear speed', 'max': 5.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'linear_sat_min', 'edit_method': '', 'default': -0.5, 'level': 0, 'min': -5.0, 'type': 'double'}, {'srcline': 290, 'description': 'Minimum input linear speed ', 'max': 5.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'linear_sat_max', 'edit_method': '', 'default': 0.5, 'level': 0, 'min': -5.0, 'type': 'double'}, {'srcline': 290, 'description': 'Invert speed signum', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'invert_speed', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 290, 'description': 'Invert steering direction', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'invert_steering', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']
