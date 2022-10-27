cmake_minimum_required(VERSION 3.0.2)
project(gruppe5)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    gazebo_ros
    tf
    geometry_msgs
    sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gruppe3_bringup
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

############### EXE #####################
add_executable(cam src/camera.cpp)
target_link_libraries(cam ${catkin_LIBRARIES})

add_executable(move_robot src/move_robot.cpp)
target_link_libraries(move_robot ${catkin_LIBRARIES})
