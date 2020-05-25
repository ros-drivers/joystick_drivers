cmake_minimum_required(VERSION 3.5)
project(wiimote_msgs)

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif ()

# generate the messages
rosidl_generate_interfaces(${PROJECT_NAME}
                           "msg/IrSourceInfo.msg"
                           "msg/TimedSwitch.msg"
                           "msg/State.msg"
                           DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
                           ADD_LINTER_TESTS
                           )

ament_export_dependencies(rosidl_default_runtime)
ament_package()
