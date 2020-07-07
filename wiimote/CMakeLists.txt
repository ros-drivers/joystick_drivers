cmake_minimum_required(VERSION 3.5)
project(wiimote)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_library(BLUETOOTH_LIB bluetooth)
if(BLUETOOTH_LIB)
  message(STATUS "Found bluetooth library.")
else()
  message(FATAL_ERROR "bluetooth library not found.")
endif()

find_library(CWIID_LIB cwiid)
if(CWIID_LIB)
  message(STATUS "Found cwiid library.")
else()
  message(FATAL_ERROR "cwiid library not found.")
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies(
  REQUIRED
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
  std_srvs
  wiimote_msgs
)

## C++ Wiimote Lib
set(wiimote_lib_HEADERS
  include/wiimote/wiimote_controller.hpp
  include/wiimote/stat_vector_3d.hpp)

set(wiimote_lib_SOURCES
  src/wiimote_controller.cpp
  src/stat_vector_3d.cpp)

ament_auto_add_library(wiimote_lib SHARED ${wiimote_lib_HEADERS} ${wiimote_lib_SOURCES})
target_link_libraries(wiimote_lib bluetooth cwiid)

rclcpp_components_register_node(
  wiimote_lib
  PLUGIN "WiimoteNode"
  EXECUTABLE wiimote_node
)
## End C++ Wiimote Lib

# C++ Teleop for Wiimote Node: Declare cpp executables
set(teleop_wiimote_HEADERS include/wiimote/teleop_wiimote.hpp)
set(teleop_wiimote_SOURCES src/teleop_wiimote.cpp)
ament_auto_add_library(teleop_wiimote SHARED ${teleop_wiimote_HEADERS} ${teleop_wiimote_SOURCES})

rclcpp_components_register_node(
  teleop_wiimote
  PLUGIN "TeleopWiimote"
  EXECUTABLE teleop_wiimote_node
)
# End C++ Teleop for Wiimote Node

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install lib
install(TARGETS wiimote_lib teleop_wiimote
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

# Install launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# Install Executables
install(TARGETS wiimote_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS nodes/feedback_tester.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
