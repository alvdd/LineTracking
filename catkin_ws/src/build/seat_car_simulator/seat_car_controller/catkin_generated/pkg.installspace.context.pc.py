# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;urdf;controller_interface;hardware_interface;control_toolbox;controller_manager;xacro;geometry_msgs;nav_msgs;std_msgs;sensor_msgs;tf;gazebo_ros_control;joint_state_controller".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lseat_car_controller".split(';') if "-lseat_car_controller" != "" else []
PROJECT_NAME = "seat_car_controller"
PROJECT_SPACE_DIR = "/home/alvaro/coche/catkin_ws/src/install"
PROJECT_VERSION = "0.0.0"
