cmake_minimum_required(VERSION 3.0.2)
project(cctag_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  sensor_msgs
  cv_bridge
)
find_package(OpenCV REQUIRED)
find_package(CCTag CONFIG REQUIRED)
set(CCTag_DIR "/usr/local/lib/cmake/CCTag/")

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES cctag_ros
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge sensor_msgs image_transport
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)



file(GLOB SOURCES
                src/cctag_detector_node.cpp
                src/cctag_detector.cpp
)
add_executable(${PROJECT_NAME}_node ${SOURCES})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} CCTag::CCTag)
