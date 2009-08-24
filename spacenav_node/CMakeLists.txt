cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(spacenav_node)
rospack_add_executable( spacenav_node
  src/spacenav_node.cpp
  )
