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
    geometry_msgs
    sensor_msgs
    tf
    gazebo_ros
    message_filters
    move_base_msgs
)
#find_package(gazebo REQUIRED)

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
add_executable(ekf src/ekf.cpp)
target_link_libraries(ekf ${catkin_LIBRARIES})

add_executable(move_rob src/move_robot.cpp)
target_link_libraries(move_rob ${catkin_LIBRARIES})
