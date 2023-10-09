cmake_minimum_required(VERSION 3.5)

project(joy)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sdl2_vendor REQUIRED)
find_package(sdl2_custom REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(joy SHARED src/joy.cpp)
target_include_directories(joy PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(joy PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  SDL2::SDL2)
target_link_libraries(joy PRIVATE
  rclcpp_components::component)
install(TARGETS joy EXPORT export_joy
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
rclcpp_components_register_node(joy
  PLUGIN "joy::Joy"
  EXECUTABLE joy_node)

add_library(game_controller SHARED src/game_controller.cpp)
target_include_directories(game_controller PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(game_controller PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  SDL2::SDL2)
target_link_libraries(game_controller PRIVATE
  rclcpp_components::component)
install(TARGETS game_controller EXPORT export_game_controller
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
rclcpp_components_register_node(game_controller
  PLUGIN "joy::GameController"
  EXECUTABLE game_controller_node)

add_executable(joy_enumerate_devices
  src/joy_enumerate_devices.cpp)
target_link_libraries(joy_enumerate_devices PRIVATE
  SDL2::SDL2)
install(TARGETS joy_enumerate_devices
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_joy)
ament_export_targets(export_game_controller)
ament_export_dependencies(
  "rclcpp"
  "sensor_msgs"
  "sdl2_vendor"
  "sdl2_custom")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
