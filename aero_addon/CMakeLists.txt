cmake_minimum_required(VERSION 2.8.3)
project(aero_addon)

find_package(catkin REQUIRED COMPONENTS
  aero_std
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES grasp_addon
 CATKIN_DEPENDS aero_std
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-std=c++11)

add_library(grasp_addon
  src/GraspAddon.cc
)
target_link_libraries(grasp_addon ${catkin_LIBRARIES})