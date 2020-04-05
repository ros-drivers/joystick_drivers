cmake_minimum_required(VERSION 3.5)
project(spacenav_node)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Load catkin and all dependencies required for this package
set(CATKIN_DEPS roscpp geometry_msgs sensor_msgs roslint)
find_package(catkin REQUIRED ${CATKIN_DEPS})
catkin_package(DEPENDS geometry_msgs sensor_msgs)

roslint_cpp()

include_directories(${catkin_INCLUDE_DIRS})
add_executable(spacenav_node src/spacenav_node.cpp)
target_link_libraries (spacenav_node spnav X11 ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()

# Install targets
install(TARGETS spacenav_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
