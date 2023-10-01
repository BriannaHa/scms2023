# cmake_minimum_required(VERSION 3.0.1)
# project(test_move)


# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   sensor_msgs
#   geometry_msgs
#   std_msgs
#   nav_msgs
#   rosbag # Needed to rosbag manipulation
#   roslib # Needed for ros::package::getPath
# )

# ## System dependencies are found with CMake's conventions
# #find_package(Boost REQUIRED COMPONENTS system signals)
# # find_package(Eigen3 REQUIRED)

# # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
# # set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
# # message ( STATUS " CMake C++ FLAGS ${CMAKE_CXX_FLAGS}")

# # ###################################
# # ## catkin specific configuration ##
# # ###################################
# # ## The catkin_package macro generates cmake config files for your package
# # ## Declare things to be passed to dependent projects
# # ## INCLUDE_DIRS: uncomment this if you package contains header files
# # ## LIBRARIES: libraries you create in this project that dependent projects also need
# # ## CATKIN_DEPENDS: catkin_packages dependent projects also need
# # ## DEPENDS: system dependencies of this project that dependent projects also need
# # catkin_package(
# #     #  INCLUDE_DIRS include
# #     #  LIBRARIES drawing_tools
# # #  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
# # #  DEPENDS system_lib
# # )

# # ###########
# # ## Build ##
# # ###########


# # Specify additional locations of header files
# # Your package locations should be listed before other locations
# include_directories(${catkin_INCLUDE_DIRS})

# # catkin_package(
# #   INCLUDE_DIRS
# #   LIBRARIES move
# #   CATKIN_DEPENDS roscpp geometry_msgs
# #   DEPENDS
# # )

# catkin_package(
#   INCLUDE_DIRS
#   LIBRARIES ${PROJECT_NAME}_laserprocessing
#   CATKIN_DEPENDS roscpp geometry_msgs
#   DEPENDS
# )

# ## Declare a C++ library
# add_library(${PROJECT_NAME}_laserprocessing src/laserprocessing.cpp)
# # add_library(controllers src/controller.cpp src/quadcopter.cpp)
# # add_library(controllers src/controller.cpp)

# ## Declare a C++ executable
# add_executable(${PROJECT_NAME}_sample src/main.cpp src/sample.cpp)

# # Add cmake target dependencies of the executable
# # This enforces generating messages before building these executables
# add_dependencies(${PROJECT_NAME}_sample ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_sample ${catkin_LIBRARIES} ${PROJECT_NAME}_laserprocessing)
# # target_link_libraries(${PROJECT_NAME}_sample ${catkin_LIBRARIES} controllers)


# ## Mark executables and/or libraries for installation
# install(TARGETS move
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# ## Mark all other useful stuff for installation
# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# #############
# ## Testing ##
# #############
# ## Add gtest based cpp test target and link libraries
# # catkin_add_gtest(${PROJECT_NAME}_test test/utest.cpp )
# # if(TARGET ${PROJECT_NAME}_test)
# #    target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} ${PROJECT_NAME}_laserprocessing )
# #    message ( STATUS " catkin_LIBRARIES ${catkin_LIBRARIES}")
# # endif()

# cmake_minimum_required(VERSION 3.0.1)
# project(test_move)

# # This pacakge depends on a5_setup, beacuse it uses a service declared there
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   sensor_msgs
#   std_msgs
#   nav_msgs
#   rosbag # Needed to rosbag manipulation
#   roslib # Needed for ros::package::getPath
#   geometry_msgs
# )

# ## System dependencies are found with CMake's conventions
# #find_package(Boost REQUIRED COMPONENTS system signals)
# find_package(Eigen3 REQUIRED)

# # NEWLY ADDED
# find_package(OpenCV REQUIRED)
# find_package(cv_bridge REQUIRED)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
# set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
# message ( STATUS " CMake C++ FLAGS ${CMAKE_CXX_FLAGS}")

# ###################################
# ## catkin specific configuration ##
# ###################################
# ## The catkin_package macro generates cmake config files for your package
# ## Declare things to be passed to dependent projects
# ## INCLUDE_DIRS: uncomment this if you package contains header files
# ## LIBRARIES: libraries you create in this project that dependent projects also need
# ## CATKIN_DEPENDS: catkin_packages dependent projects also need
# ## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
#     #  INCLUDE_DIRS include
#     #  LIBRARIES drawing_tools
# #  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
# #  DEPENDS system_lib
# )

# ###########
# ## Build ##
# ###########


# # Specify additional locations of header files
# # Your package locations should be listed before other locations
# include_directories(
#   include
#   src
#   ${catkin_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIR}
#   ${OpenCV_INCLUDE_DIRS}
# )


# ## Declare a C++ library
# add_library(${PROJECT_NAME}_laserprocessing src/laserprocessing.cpp)
# # add_library(controllers src/controller.cpp src/quadcopter.cpp)
# # add_library(controllers src/controller.cpp)

# ## Declare a C++ executable
# add_executable(${PROJECT_NAME}_sample src/main.cpp src/sample.cpp)

# # Add cmake target dependencies of the executable
# # This enforces generating messages before building these executables
# add_dependencies(${PROJECT_NAME}_sample ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_sample ${catkin_LIBRARIES} ${PROJECT_NAME}_laserprocessing)
# # target_link_libraries(${PROJECT_NAME}_sample ${OpenCV_LIBS} ${PROJECT_NAME}_laserprocessing)
# # target_link_libraries(${PROJECT_NAME}_sample ${catkin_LIBRARIES} ${PROJECT_NAME}_laserprocessing cv_bridge)

# # target_link_libraries(${PROJECT_NAME}_sample ${OpenCV_LIBS} ${catkin_LIBRARIES} ${PROJECT_NAME}_laserprocessing cv_bridge) # NEWLY ADDED
# # target_link_libraries(${PROJECT_NAME}_sample ${catkin_LIBRARIES} controllers)

# # #############
# # ## Testing ##
# # #############
# # ## Add gtest based cpp test target and link libraries
# # catkin_add_gtest(${PROJECT_NAME}_test test/utest.cpp )
# # if(TARGET ${PROJECT_NAME}_test)
# #    target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} ${PROJECT_NAME}_laserprocessing )
# #    message ( STATUS " catkin_LIBRARIES ${catkin_LIBRARIES}")
# # endif()

# cmake_minimum_required(VERSION 3.0.1)
# project(test_move)


# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   sensor_msgs
#   geometry_msgs
#   std_msgs
#   nav_msgs
#   rosbag # Needed to rosbag manipulation
#   roslib # Needed for ros::package::getPath
# )

# ## System dependencies are found with CMake's conventions
# #find_package(Boost REQUIRED COMPONENTS system signals)
# # find_package(Eigen3 REQUIRED)

# # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
# # set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
# # message ( STATUS " CMake C++ FLAGS ${CMAKE_CXX_FLAGS}")

# # ###################################
# # ## catkin specific configuration ##
# # ###################################
# # ## The catkin_package macro generates cmake config files for your package
# # ## Declare things to be passed to dependent projects
# # ## INCLUDE_DIRS: uncomment this if you package contains header files
# # ## LIBRARIES: libraries you create in this project that dependent projects also need
# # ## CATKIN_DEPENDS: catkin_packages dependent projects also need
# # ## DEPENDS: system dependencies of this project that dependent projects also need
# # catkin_package(
# #     #  INCLUDE_DIRS include
# #     #  LIBRARIES drawing_tools
# # #  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
# # #  DEPENDS system_lib
# # )

# # ###########
# # ## Build ##
# # ###########


# # Specify additional locations of header files
# # Your package locations should be listed before other locations
# include_directories(${catkin_INCLUDE_DIRS})

# # catkin_package(
# #   INCLUDE_DIRS
# #   LIBRARIES move
# #   CATKIN_DEPENDS roscpp geometry_msgs
# #   DEPENDS
# # )

# catkin_package(
#   INCLUDE_DIRS
#   LIBRARIES ${PROJECT_NAME}_laserprocessing
#   CATKIN_DEPENDS roscpp geometry_msgs
#   DEPENDS
# )

# ## Declare a C++ library
# add_library(${PROJECT_NAME}_laserprocessing src/laserprocessing.cpp)
# # add_library(controllers src/controller.cpp src/quadcopter.cpp)
# # add_library(controllers src/controller.cpp)

# ## Declare a C++ executable
# add_executable(${PROJECT_NAME}_sample src/main.cpp src/sample.cpp)

# # Add cmake target dependencies of the executable
# # This enforces generating messages before building these executables
# add_dependencies(${PROJECT_NAME}_sample ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_sample ${catkin_LIBRARIES} ${PROJECT_NAME}_laserprocessing)
# # target_link_libraries(${PROJECT_NAME}_sample ${catkin_LIBRARIES} controllers)


# ## Mark executables and/or libraries for installation
# install(TARGETS move
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# ## Mark all other useful stuff for installation
# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# #############
# ## Testing ##
# #############
# ## Add gtest based cpp test target and link libraries
# # catkin_add_gtest(${PROJECT_NAME}_test test/utest.cpp )
# # if(TARGET ${PROJECT_NAME}_test)
# #    target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} ${PROJECT_NAME}_laserprocessing )
# #    message ( STATUS " catkin_LIBRARIES ${catkin_LIBRARIES}")
# # endif()

cmake_minimum_required(VERSION 3.0.1)
project(test_move)

# This pacakge depends on a5_setup, beacuse it uses a service declared there
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  rosbag # Needed to rosbag manipulation
  roslib # Needed for ros::package::getPath
  geometry_msgs
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system signals)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
message ( STATUS " CMake C++ FLAGS ${CMAKE_CXX_FLAGS}")

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES drawing_tools
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########


# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_laserprocessing src/laserprocessing.cpp)
# add_library(controllers src/controller.cpp src/quadcopter.cpp)
# add_library(controllers src/controller.cpp)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_sample src/main.cpp src/sample.cpp)

# Add cmake target dependencies of the executable
# This enforces generating messages before building these executables
add_dependencies(${PROJECT_NAME}_sample ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_sample ${catkin_LIBRARIES} ${PROJECT_NAME}_laserprocessing)
# target_link_libraries(${PROJECT_NAME}_sample ${catkin_LIBRARIES} controllers)

# #############
# ## Testing ##
# #############
# ## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}_test test/utest.cpp )
# if(TARGET ${PROJECT_NAME}_test)
#    target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} ${PROJECT_NAME}_laserprocessing )
#    message ( STATUS " catkin_LIBRARIES ${catkin_LIBRARIES}")
# endif()
