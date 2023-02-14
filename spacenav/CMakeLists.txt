cmake_minimum_required(VERSION 3.5)

project(spacenav)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(SPNAV REQUIRED)

add_library(spacenav
  SHARED
    src/spacenav.cpp)

target_include_directories(spacenav PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${spnav_INCLUDE_DIR})

target_link_libraries(spacenav PUBLIC
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  spnav
)

target_link_libraries(spacenav PRIVATE
  rclcpp_components::component
)

install(TARGETS spacenav EXPORT export_spacenav
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

rclcpp_components_register_node(spacenav
  PLUGIN "spacenav::Spacenav"
  EXECUTABLE spacenav_node)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_targets(export_spacenav)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
