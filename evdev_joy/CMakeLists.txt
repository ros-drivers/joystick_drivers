cmake_minimum_required(VERSION 2.8.3)
project(evdev_joy)

add_compile_options(-std=c++11)

find_package(PkgConfig REQUIRED)
pkg_search_module(LIBEVDEV REQUIRED libevdev)
if(LIBEVDEV_FOUND)
  include_directories(SYSTEM ${LIBEVDEV_INCLUDE_DIRS})
  link_directories(${LIBEVDEV_LIBRARY_DIRS})
else()
  message( FATAL_ERROR "Failed to find libevdev" )
endif(LIBEVDEV_FOUND)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
catkin_package()



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(evdev_joystick_node src/evdev.cpp)
target_link_libraries(evdev_joystick_node -levdev ${catkin_LIBRARIES})

# Install targets
install(TARGETS evdev_joystick_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
