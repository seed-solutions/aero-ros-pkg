#if(NOT USE_ROSBUILD)
#  include(catkin.cmake)
#  return()
#endif()

cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_aero_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp tf trajectory_msgs geometry_msgs nav_msgs pr2_controllers_msgs
)
find_package(Boost REQUIRED COMPONENTS filesystem system signals thread)

catkin_package(
  CATKIN_DEPENDS
  roscpp tf trajectory_msgs geometry_msgs nav_msgs pr2_controllers_msgs
  DEPENDS
  INCLUDE_DIRS include
  LIBRARIES aero_controller
)
include_directories(include ${Boost_INCLUDE_DIRS})
add_library(aero_controller src/aero_controller.cpp)
target_link_libraries(aero_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(aero_controller_standalone src/aero_controller_standalone.cpp)
target_link_libraries(aero_controller_standalone aero_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(aero_controller_node src/aero_controller_node.cpp)
target_link_libraries(aero_controller_node aero_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})
