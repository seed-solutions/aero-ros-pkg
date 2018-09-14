cmake_minimum_required(VERSION 2.8.3)
project(aero_teleop)

find_package(catkin REQUIRED roscpp aero_std)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ps_teleop
  )

include_directories(include ${catkin_INCLUDE_DIRS})
add_definitions(-std=c++11)

include_directories(${aero_std_INCLUDE_DIRS})

add_library(ps_teleop src/ps_teleop.cc)
target_link_libraries(ps_teleop ${catkin_LIBRARIES} ${aero_std_LIBRARIES})

add_executable(ps_teleop_node src/ps_teleop_node.cc)
target_link_libraries(ps_teleop_node ps_teleop)
