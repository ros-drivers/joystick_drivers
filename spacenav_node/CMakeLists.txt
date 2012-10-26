cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

rosbuild_add_executable(spacenav_node src/spacenav_node.cpp)
target_link_libraries (spacenav_node spnav X11)
