cmake_minimum_required(VERSION 3.0.2)
project(interactive_controller)

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
  controller_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs controller_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

###########
## Build ##
###########

include_directories(src ../include)
file(GLOB SOURCES src/interactive_controller.cpp)
add_executable(${PROJECT_NAME}_node ${SOURCES})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
