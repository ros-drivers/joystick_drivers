cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_genmsg()

# Look for <linux/joystick.h>
include(CheckIncludeFiles)
check_include_files(linux/joystick.h HAVE_LINUX_JOYSTICK_H)
if(HAVE_LINUX_JOYSTICK_H)
  include_directories(msg/cpp)
  rosbuild_add_executable(joy_node joy_node.cpp)
else(HAVE_LINUX_JOYSTICK_H)
  message("Warning: no <linux/joystick.h>; won't build joy node")
endif(HAVE_LINUX_JOYSTICK_H)

rosbuild_add_pyunit(test/test_joy_msg_migration.py)
