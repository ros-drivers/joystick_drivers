cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(libspnav_include_dir this_is_not_needed)
set(libspnav_library this_is_not_needed)

rosbuild_add_executable(spacenav_node src/spacenav_node.cpp)
target_link_libraries (spacenav_node spnav X11)
