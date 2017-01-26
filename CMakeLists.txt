cmake_minimum_required(VERSION 2.8.3)
project(aero_std)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf
  cmake_modules
  moveit_core moveit_ros_planning moveit_ros_planning_interface
)

find_package(Eigen REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES aero_interface aero_moveit_interface
 CATKIN_DEPENDS
 geometry_msgs roscpp roslib sensor_msgs std_msgs tf
 moveit_core moveit_ros_planning moveit_ros_planning_interface
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_definitions(-std=c++11)

add_library(aero_interface
  src/AeroInterface.cc
)

add_library(aero_moveit_interface
  src/AeroMoveitInterface.cc
  src/AeroInterface.cc
)
