cmake_minimum_required(VERSION 3.0.2)
project(pid_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  sensor_msgs
  cv_bridge
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES offboard
 CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

add_library(pid src/pid.cpp)

# catkin_add_nosetests(test)
# define HILT with mode simulation hardware

# Find Gazebo
# include_directories( src ../../pid)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

add_library(${PROJECT_NAME}
  src/offboard_control.cpp 
  src/pid.cpp
)

file(GLOB SOURCES src/offboard_control.cpp src/pid.cpp)
add_executable(pid_controller_node src/offboard_control.cpp src/pid.cpp)
target_link_libraries(pid_controller_node ${catkin_LIBRARIES} pid_controller)

add_executable(landing_service
  src/service_land.cpp
)
target_link_libraries(landing_service ${PROJECT_NAME} ${catkin_LIBRARIES})


# Build our plugin

#target_link_libraries(off_node_plugin ${GAZEBO_LIBRARIES})



