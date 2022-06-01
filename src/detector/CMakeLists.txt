cmake_minimum_required(VERSION 3.0.2)
project(detector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        cv_bridge image_geometry geometry_msgs roscpp rospy std_msgs tf2 image_transport std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS cv_bridge image_geometry geometry_msgs std_msgs roscpp rospy std_msgs tf2 image_transport)

###########
## Build ##
###########

add_definitions(-DSITL)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

# add_executable(aruco_detector_node src/aruco_detector_node.cpp)
# target_link_libraries(aruco_detector_node ${catkin_LIBRARIES})
