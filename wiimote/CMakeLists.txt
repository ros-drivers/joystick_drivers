# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(wiimote)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# View the makefile commands during build
#set(CMAKE_VERBOSE_MAKEFILE on)

# Set compile flags
set(CMAKE_CXX_FLAGS "-fPIE -fPIC -std=c++11 -O2 -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")
# Flags executables
set(CMAKE_EXE_LINKER_FLAGS "-pie -z noexecstack -z relro -z now")
# Flags shared libraries
set(CMAKE_SHARED_LINKER_FLAGS "-z noexecstack -z relro -z now ${CMAKE_SHARED_LINKER_FLAGS}")

# Load catkin and all dependencies required for this package
set(CATKIN_DEPS geometry_msgs sensor_msgs std_msgs std_srvs rospy roscpp roslib genmsg)
find_package(catkin REQUIRED ${CATKIN_DEPS} roslint)

catkin_python_setup()

# generate the messages
add_message_files(
  DIRECTORY msg
  FILES
  IrSourceInfo.msg
  State.msg
  TimedSwitch.msg)

generate_messages(DEPENDENCIES geometry_msgs std_msgs sensor_msgs)

# ROS Lint the code
roslint_cpp()
#roslint_python()  # TODO(jbohren): ROS Lint the Python code
roslint_add_test()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPS}
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## C++ Wiimote Node: Declare cpp executables
add_executable(wiimote_node src/wiimote_controller.cpp src/stat_vector_3d.cpp)
target_link_libraries(wiimote_node ${catkin_LIBRARIES} bluetooth cwiid)
add_dependencies(wiimote_node wiimote_generate_messages_cpp)
## End of C++ Wiimote Node

## C++ Teleop for Wiimote Node: Declare cpp executables
add_executable(teleop_wiimote src/teleop_wiimote.cpp)
target_link_libraries(teleop_wiimote ${catkin_LIBRARIES})
add_dependencies(teleop_wiimote wiimote_generate_messages_cpp)
## End C++ Teleop for Wiimote Node

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install targets
install(PROGRAMS nodes/feedbackTester.py  nodes/wiimote_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
## Mark executables and/or libraries for installation
install(TARGETS wiimote_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
