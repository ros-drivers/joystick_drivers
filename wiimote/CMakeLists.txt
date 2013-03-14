# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(wiimote)

# Load catkin and all dependencies required for this package
set(CATKIN_DEPS geometry_msgs sensor_msgs std_msgs std_srvs rospy roslib genmsg)
set(ROSDEP_DEPS python-numpy python-cwiid)
find_package(catkin REQUIRED ${CATKIN_DEPS})

catkin_python_setup()

# generate the messages
add_message_files(
  DIRECTORY msg
  FILES
  IrSourceInfo.msg
  State.msg
  TimedSwitch.msg)

generate_messages(DEPENDENCIES geometry_msgs std_msgs sensor_msgs)

# define the package
catkin_package(DEPENDS ${CATKIN_DEPS})

# Install targets
install(PROGRAMS nodes/feedbackTester.py  nodes/wiimote_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
