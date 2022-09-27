cmake_minimum_required(VERSION 3.0.2)
project(detector_switch_ros)

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
  image_geometry
  geometry_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)
set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aruco
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge sensor_msgs image_transport geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB SOURCES
                src/detector_switch_node.cpp
                src/detector_switch.cpp
)

# file(GLOB SOURCES_1
#                 src/transform_adaption_node.cpp
#                 src/transform_adaption.cpp
# )

file(GLOB SOURCES_2
                src/aruco_detector.cpp
)

add_executable(${PROJECT_NAME}_node ${SOURCES})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

# add_executable(transform_adaption_node ${SOURCES_1})
# target_link_libraries(transform_adaption_node ${catkin_LIBRARIES})

add_executable(aruco_detector_node ${SOURCES_2})
target_link_libraries(aruco_detector_node ${catkin_LIBRARIES})

add_executable(cam_info_node src/cam_info.cpp)
target_link_libraries(cam_info_node ${catkin_LIBRARIES})
