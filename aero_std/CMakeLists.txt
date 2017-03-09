cmake_minimum_required(VERSION 2.8.3)
project(aero_std)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslib
  actionlib
  actionlib_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
  cmake_modules
  moveit_core moveit_ros_planning moveit_ros_planning_interface
  aero_startup
  visualization_msgs
)

find_package(Eigen REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES aero_interface aero_moveit_interface object_features
 CATKIN_DEPENDS
 geometry_msgs roscpp roslib sensor_msgs std_msgs std_srvs tf actionlib actionlib_msgs
 moveit_core moveit_ros_planning moveit_ros_planning_interface aero_startup visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

#link_directories(${catkin_LIBRARY_DIRS})

add_definitions(-std=c++11)

add_library(aero_interface
  src/AeroInterface.cc
)
target_link_libraries(aero_interface ${catkin_LIBRARIES})

add_library(aero_moveit_interface
  src/AeroMoveitInterface.cc
  src/AeroInterface.cc
)
target_link_libraries(aero_moveit_interface ${catkin_LIBRARIES})

add_library(object_features
  src/ObjectFeatures.cc
)
target_link_libraries(object_features ${catkin_LIBRARIES})