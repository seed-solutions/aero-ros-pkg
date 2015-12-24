cmake_minimum_required(VERSION 2.8.3)
project(aero_startup)

# Dependencies

# catkin ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp tf std_msgs sensor_msgs
  trajectory_msgs geometry_msgs nav_msgs pr2_controllers_msgs
  move_base_msgs
  message_generation
)
if(NOT catkin_LIBRARIES)
  message(FATAL "catkin required but not supported")
endif()

# boost
find_package(Boost REQUIRED COMPONENTS filesystem system signals thread)
if(NOT Boost_INCLUDE_DIRS)
  message(FATAL "boost required but not supported")
else()
  include_directories(include ${Boost_INCLUDE_DIRS})
endif()

# pcl
find_package(PCL 1.7 REQUIRED)
if(NOT PCL_INCLUDE_DIRS)
  message(WARNING "PCL not found, disabled")
else()
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  set(FOUND_PCL 1)
endif()

# opencv
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
  message(WARNING "opencv not found, disabled")
else()
  include_directories(${OpenCV_INCLUDE_DIRS})
  set(FOUND_OpenCV 1)
endif()

# libsvm
set(FOUND_LIBSVM)
if(FOUND_LIBSVM)
  message(STATUS "found libsvm")
else()
  message(WARNING "libsvm not found, disabled")
endif()



# ROS setups

set(GENERATE_SRV)
if(GENERATE_SRV)
  # auto-add services
  add_service_files(
    FILES
  )
  generate_messages(
    DEPENDENCIES
    std_msgs
  )
endif()

catkin_package(
  CATKIN_DEPENDS
  roscpp tf std_msgs sensor_msgs
  trajectory_msgs geometry_msgs nav_msgs pr2_controllers_msgs move_base_msgs
  DEPENDS
  INCLUDE_DIRS
  LIBRARIES aero_hardware_interface
)

include_directories(${aero_startup_SOURCE_DIR})


# >>> add dependencies
# <<< add dependencies

# Executables

# >>> add controllers
# <<< add controllers

# >>> add applications
# <<< add applications
