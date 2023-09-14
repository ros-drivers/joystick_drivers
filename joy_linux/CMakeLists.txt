cmake_minimum_required(VERSION 3.5)

project(joy_linux)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# for now, this program only exists for Linux hosts, since it calls into
# the underlying joystick driver provided by the Linux kernel.
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  # Look for <linux/joystick.h>
  include(CheckIncludeFiles)
  check_include_files(linux/joystick.h HAVE_LINUX_JOYSTICK_H)

  if(HAVE_LINUX_JOYSTICK_H)
    include_directories(msg/cpp ${catkin_INCLUDE_DIRS})
    add_executable(joy_linux_node src/joy_linux_node.cpp)
    target_link_libraries(joy_linux_node PRIVATE
      ${diagnostic_msgs_TARGETS}
      diagnostic_updater::diagnostic_updater
      rclcpp::rclcpp
      ${sensor_msgs_TARGETS}
    )
    install(TARGETS joy_linux_node DESTINATION lib/${PROJECT_NAME})

    if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      set(ament_cmake_copyright_FOUND TRUE)
      ament_lint_auto_find_test_dependencies()
    endif()

  else()
    message(FATAL_ERROR "no <linux/joystick.h>; can't build joy node")
  endif()
endif()

ament_package()
