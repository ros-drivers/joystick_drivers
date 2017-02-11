# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(spacenav_node)

# Load catkin and all dependencies required for this package
set(CATKIN_DEPS roscpp geometry_msgs sensor_msgs)
find_package(catkin REQUIRED ${CATKIN_DEPS})
catkin_package(DEPENDS ${CATKIN_DEPS})

include_directories(${catkin_INCLUDE_DIRS})
add_executable(spacenav_node src/spacenav_node.cpp)
target_link_libraries (spacenav_node spnav X11 ${catkin_LIBRARIES})

# Install targets
install(TARGETS spacenav_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )

install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
