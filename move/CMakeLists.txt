cmake_minimum_required(VERSION 2.8.3)
project(move)

## Find catkin macros and libraries
# find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs joy)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  rosbag # Needed to rosbag manipulation
  roslib # Needed for ros::package::getPath
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES move
  CATKIN_DEPENDS roscpp geometry_msgs
  DEPENDS
)

###########
## Build ##
###########

add_executable(move src/move.cpp)
target_link_libraries(move ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
#install(PROGRAMS
#  scripts/move
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
install(TARGETS move
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other useful stuff for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
