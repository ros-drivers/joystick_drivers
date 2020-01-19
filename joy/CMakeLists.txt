cmake_minimum_required(VERSION 3.5)
project(joy)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# for now, this program only exists for Linux hosts, since it calls into
# the underlying joystick driver provided by the Linux kernel.
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  # Look for <linux/joystick.h>
  include(CheckIncludeFiles)
  check_include_files(linux/joystick.h HAVE_LINUX_JOYSTICK_H)

  if(HAVE_LINUX_JOYSTICK_H)
    add_library(
      ${PROJECT_NAME}_node SHARED
      src/joy_node.cpp
    )
    target_compile_definitions(
      ${PROJECT_NAME}_node
      PRIVATE "COMPOSITION_BUILDING_DLL"
    )
    ament_target_dependencies(joy_node
      "diagnostic_updater"
      "rclcpp"
      "rclcpp_components"
      "sensor_msgs"
    )
    rclcpp_components_register_nodes(
      ${PROJECT_NAME}_node
      "joy::JoyNode"
    )
    ament_export_interfaces(export_${PROJECT_NAME}_node HAS_LIBRARY_TARGET)

    install(DIRECTORY include/
      DESTINATION include
    )

    install(TARGETS ${PROJECT_NAME}_node
      EXPORT export_${PROJECT_NAME}_node
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
      INCLUDES DESTINATION include
    )
  else()
    message("Warning: no <linux/joystick.h>; won't build joy node")
  endif()

  # install(
  #   DIRECTORY
  #     config
  #     launch
  #     scripts
  #   DESTINATION share
  # )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
