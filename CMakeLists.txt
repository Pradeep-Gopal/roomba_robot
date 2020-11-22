cmake_minimum_required(VERSION 2.8.3)
project(roomba_robot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(
  catkin REQUIRED COMPONENTS 
  roscpp
  sensor_msgs
  geometry_msgs
  move_base_msgs
  std_msgs
  rospy
)

## Declare a catkin package
catkin_package()

## Build target library
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(roomba_robot)
target_link_libraries(roomba_robot ${catkin_LIBRARIES})
