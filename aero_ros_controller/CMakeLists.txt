cmake_minimum_required(VERSION 2.8.3)
project(aero_ros_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  control_toolbox
  controller_manager
  hardware_interface
  transmission_interface
  joint_limits_interface
  angles
  urdf
  aero_startup
#  pluginlib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_interface
  CATKIN_DEPENDS
    roscpp
    std_msgs
    controller_manager
    control_toolbox
    hardware_interface
    transmission_interface
    joint_limits_interface
    angles
    urdf
    aero_startup
    #    pluginlib
)

find_package(Eigen3 REQUIRED)
##find_package(Boost REQUIRED COMPONENTS thread)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# add c++ dependencies :
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c+11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  add_definitions(-DCXX11_SUPPORTED)
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  add_definitions(-DCXX11_SUPPORTED)
else()
  message(FATAL "c++11 required but not supported")
endif()

## Executable
add_executable(${PROJECT_NAME} src/aero_ros_controller.cpp src/aero_robot_hardware.cpp src/AeroMoveBaseRH.cc)
target_link_libraries(${PROJECT_NAME} aero_controllers ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## Library
add_library(robot_interface src/RobotInterface.cc )
target_link_libraries(robot_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## Install
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

#install(FILES robot_hw_sim_plugins.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

####
add_executable(aero_interface_test src/AeroInterfaceTest.cc)
target_link_libraries(aero_interface_test ${catkin_LIBRARIES} ${Boost_LIBRARIES} robot_interface )

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_robot_interface test/test_robot_interface.test test/test_robot_interface.cpp)
  target_link_libraries(test_robot_interface ${catkin_LIBRARIES} ${GTEST_LIBRARIES} robot_interface )

endif() ## CATKIN_ENABLE_TESTING
