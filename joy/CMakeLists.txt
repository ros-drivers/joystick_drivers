cmake_minimum_required(VERSION 3.5)
project(joy)

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
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# for now, this program only exists for Linux hosts, since it calls into
# the underlying joystick driver provided by the Linux kernel.
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  # Look for <linux/joystick.h>
  include(CheckIncludeFiles)
  check_include_files(linux/joystick.h HAVE_LINUX_JOYSTICK_H)

  if(HAVE_LINUX_JOYSTICK_H)
    add_executable(
      ${PROJECT_NAME}_node
      src/joy_node.cpp
      src/joystick_data.cpp
      src/linux_force_feedback.cpp
      src/linux_helper.cpp
      src/linux_joystick.cpp
    )
    ament_target_dependencies(${PROJECT_NAME}_node
      "diagnostic_updater"
      "rclcpp"
      #"rclcpp_components"
      "sensor_msgs"
    )
    
    install(TARGETS ${PROJECT_NAME}_node
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
      INCLUDES DESTINATION include
    )
  else()
    message("Warning: no <linux/joystick.h>; won't build joy node")
  endif()
else()
  message("Warning: for now, cannot build joy node for other systems than Linux")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
