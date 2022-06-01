cmake_minimum_required(VERSION 3.0.2)
project(transform_adaption)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        geometry_msgs roscpp rospy std_msgs tf2 mav_planning_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS geometry_msgs std_msgs roscpp rospy std_msgs tf2)

catkin_install_python(PROGRAMS 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
)
include_directories(
        include
        ../include
)
file(GLOB SOURCES
                src/transform_adaption_node.cpp
                src/transform_adaption.cpp
                src/generate_curve.cpp
)
add_executable(${PROJECT_NAME}_node ${SOURCES})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
