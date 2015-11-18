cmake_minimum_required(VERSION 2.8.3)
project(aero_startup)

# Dependencies

# c++11/c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c+11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(FATAL "c++11 required but not supported")
endif()

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

# eigen
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIR)
  message(WARNING "Eigen3 not found, disabled")
else()
  include_directories(${EIGEN3_INCLUDE_DIR})
  set(FOUND_EIGEN 1)
endif()

# pcl
find_package(PCL 1.3 REQUIRED)
if(NOT PCL_INCLUDE_DIRS)
  message(WARNING "PCL not found, disabled")
else()
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  set(FOUND_PCL 1)
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



# Executables

add_executable(aero_controller_node
  aero_common/Angle2Stroke.cc
  aero_common/Stroke2Angle.cc
  aero_common/AngleJointNames.cc
  aero_hardware_interface/AeroControllers.cc
  aero_hardware_interface/AeroControllerProto.cc
  aero_hardware_interface/AeroControllerNode.cc
  aero_hardware_interface/Main.cc)
target_link_libraries(aero_controller_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(aero_joint_state_publisher
  aero_controller_manager/AeroJointStatePublisher.cc)
target_link_libraries(aero_joint_state_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# >>> add controllers
# <<< add controllers

if(FOUND_LIBSVM)
  # auto-add executables from aero_object_manipulation/manipulation/configure
endif()

if(FOUND_EIGEN AND FOUND_PCL)
  add_executable(extract_object_features
    aero_object_manipulation/perception/ExtractObjectFeatures.cc)
  target_link_libraries(extract_object_features ${PCL_LIBRARIES}
    ${catkin_LIBRARIES} ${Boost_LIBRARIES})
endif()