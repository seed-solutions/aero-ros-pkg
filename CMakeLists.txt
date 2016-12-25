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

################## optional: ##################

find_package(PCL 1.8)
if(NOT PCL_INCLUDE_DIRS)
  message(WARNING "PCL not found, disabled")
else()
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  set(FOUND_PCL 1)
endif()

find_package(OpenCV 3)
if(NOT OpenCV_FOUND)
  message(WARNING "opencv not found, disabled")
else()
  include_directories(${OpenCV_INCLUDE_DIRS})
  message(STATUS "OpenCV library status:")
  message(STATUS "    version: ${OpenCV_VERSION}")
  message(STATUS "    libraries: ${OpenCV_LIBS}")
  message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
  set(FOUND_OpenCV 1)
endif()

if (FOUND_OpenCV AND FOUND_PCL)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES aero_interface aero_moveit_interface aerocv
 CATKIN_DEPENDS
 geometry_msgs roscpp roslib sensor_msgs std_msgs tf
 moveit_core moveit_ros_planning moveit_ros_planning_interface
)

################## :optional ##################

else()
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES aero_interface aero_moveit_interface
 CATKIN_DEPENDS
 geometry_msgs roscpp roslib sensor_msgs std_msgs tf
 moveit_core moveit_ros_planning moveit_ros_planning_interface
)
endif()

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
)

################## optional: ##################

if (FOUND_OpenCV AND FOUND_PCL)
  add_library(aerocv
    src/aerocv.cc
  )
endif()

################## :optional ##################
