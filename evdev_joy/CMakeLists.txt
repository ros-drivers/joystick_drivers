cmake_minimum_required(VERSION 2.8.3)
project(evdev_joy)


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)
catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(evdev_joystick_node src/evdev.cpp)
target_link_libraries(evdev_joystick_node -levdev ${catkin_LIBRARIES})
