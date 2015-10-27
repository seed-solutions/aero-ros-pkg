cmake_minimum_required(VERSION 2.8.3)
project(aero_startup)

# check c++11/c++0x
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

find_package(catkin REQUIRED COMPONENTS
  roscpp tf std_msgs sensor_msgs trajectory_msgs geometry_msgs nav_msgs pr2_controllers_msgs
)
find_package(Boost REQUIRED COMPONENTS filesystem system signals thread)

catkin_package(
  CATKIN_DEPENDS
  roscpp tf std_msgs sensor_msgs trajectory_msgs geometry_msgs nav_msgs pr2_controllers_msgs
  DEPENDS
  INCLUDE_DIRS
  LIBRARIES aero_hardware_interface
)
include_directories(include ${Boost_INCLUDE_DIRS})

include_directories(${aero_startup_SOURCE_DIR})

add_executable(aero_controller_node
  aero_common/Angle2Stroke.cc
  aero_common/Stroke2Angle.cc
  aero_common/AngleJointNames.cc
  aero_hardware_interface/AeroControllers.cc
  aero_hardware_interface/AeroControllerProto.cc
  aero_hardware_interface/AeroControllerMain.cc
  aero_hardware_interface/AeroControllerNode.cc)
target_link_libraries(aero_controller_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})