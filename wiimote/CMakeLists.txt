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

include(wiimote-extras.cmake)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(wiimote_msgs REQUIRED)

## C++ Wiimote Lib
add_library(wiimote_lib SHARED
  src/wiimote_controller.cpp
  src/stat_vector_3d.cpp)
target_include_directories(wiimote_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(wiimote_lib PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${wiimote_msgs_TARGETS}
  wiimote::bluetooth
  wiimote::cwiid)
target_link_libraries(wiimote_lib PRIVATE
  rclcpp_components::component)

rclcpp_components_register_node(
  wiimote_lib
  PLUGIN "WiimoteNode"
  EXECUTABLE wiimote_node
)
## End C++ Wiimote Lib

# C++ Teleop for Wiimote Node: Declare cpp executables
add_library(teleop_wiimote SHARED
  src/teleop_wiimote.cpp)
target_include_directories(teleop_wiimote PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(teleop_wiimote PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${wiimote_msgs_TARGETS})
target_link_libraries(teleop_wiimote PRIVATE
  rclcpp_components::component)

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
install(TARGETS wiimote_lib teleop_wiimote EXPORT export_wiimote
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# Install launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# Install Executables
install(TARGETS wiimote_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS nodes/feedback_tester.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_wiimote)
ament_export_dependencies(
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "std_srvs"
  "wiimote"
)

ament_package(
  CONFIG_EXTRAS "wiimote-extras.cmake"
)
