cmake_minimum_required(VERSION 2.8.3)
project(aero_std)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslib
  rospy
  actionlib
  actionlib_msgs
  sensor_msgs
  std_msgs
  std_srvs
  nav_msgs
  tf
  cmake_modules
  moveit_core moveit_ros_planning moveit_ros_planning_interface
  aero_startup
  aero_ros_controller
  visualization_msgs
  message_generation

  control_msgs
)

find_package(Eigen3 REQUIRED)

# add yaml-cpp dependencies :
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
add_definitions(-DNEW_YAMLCPP)
endif()

add_service_files(
  FILES
  SaveSpot.srv
  GetSpots.srv
  GetSpot.srv
  DeleteSpot.srv
  )
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  )

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES aero_moveit_interface object_features spot_list
 CATKIN_DEPENDS
 geometry_msgs roscpp roslib sensor_msgs std_msgs std_srvs tf actionlib actionlib_msgs
 moveit_core moveit_ros_planning moveit_ros_planning_interface aero_startup visualization_msgs aero_ros_controller
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

#link_directories(${catkin_LIBRARY_DIRS})

# check ros_distro
if("$ENV{ROS_DISTRO}" STRGREATER "jade")
  add_definitions(-DKINETIC)
endif()

add_definitions(-std=c++11)

add_library(aero_moveit_interface
  src/AeroMoveitInterface.cc
)
target_link_libraries(aero_moveit_interface ${catkin_LIBRARIES})
add_dependencies(aero_moveit_interface ${PROJECT_NAME}_gencpp)

add_library(spot_list src/spot_list.cc)
target_link_libraries(spot_list ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(spot_manager src/spot_manager.cc)
target_link_libraries(spot_manager ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} spot_list)
add_dependencies(spot_manager ${PROJECT_NAME}_gencpp)

catkin_add_gtest(test_spot test/test_spot.cc)
target_link_libraries(test_spot ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} spot_list)

add_executable(look_at src/look_at.cc)
target_link_libraries(look_at ${catkin_LIBRARIES} aero_moveit_interface)

add_library(object_features
  src/ObjectFeatures.cc
)
target_link_libraries(object_features ${catkin_LIBRARIES} aero_moveit_interface)

###
#add_library(robot_interface src/RobotInterface.cc)
#target_link_libraries(robot_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})

##add_executable(aero_moveit_test src/AeroMoveitTest.cc)
##target_link_libraries(aero_moveit_test ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
