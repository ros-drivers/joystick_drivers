# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(ps3joy)

# Load catkin and all dependencies required for this package
set(CATKIN_DEPS diagnostic_msgs sensor_msgs rospy rosgraph)
set(ROSDEP_DEPS libusb-dev joystick bluez python-bluez)
find_package(catkin REQUIRED ${CATKIN_DEPS})
catkin_package(DEPENDS ${CATKIN_DEPS} ${ROSDEP_DEPS})

include_directories(${catkin_INCLUDE_DIRS})
add_executable(sixpair sixpair.c)
target_link_libraries(sixpair -lusb ${catkin_LIBRARIES})
