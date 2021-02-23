cmake_minimum_required(VERSION 3.5)
project(spacenav_node)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_library(spacenav_node
  src/spacenav_node.cpp)
target_include_directories(spacenav_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  spacenav_node
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "sensor_msgs"
)
target_link_libraries(spacenav_node spnav X11)
rclcpp_components_register_node(spacenav_node
  PLUGIN "spacenav_node::SpacenavNode"
  EXECUTABLE spacenav_node_exec)

target_compile_definitions(spacenav_node PRIVATE "ROS2_TEST_BASE_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS spacenav_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  spacenav_node
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
