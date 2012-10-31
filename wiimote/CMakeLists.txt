# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)

# Load catkin and all dependencies required for this package
set(CATKIN_DEPS geometry_msgs sensor_msgs std_msgs std_srvs rospy roslib)
set(ROSDEP_DEPS python-numpy python-cwiid)
find_package(catkin REQUIRED ${CATKIN_DEPS})
catkin_package(
  DEPENDS ${CATKIN_DEPS} ${ROSDEP_DEPS}
  #INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME})
