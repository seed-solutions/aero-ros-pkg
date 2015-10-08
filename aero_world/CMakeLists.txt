cmake_minimum_required(VERSION 2.8.3)
project(aero_world)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# check c++11/c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c+11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(WARNING "c++11 required but not supported")
endif()

# find eigen
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIR)
  message(WARNING "Eigen3 not found")
else()
  message(STATUS "found Eigen3")
  include_directories(${EIGEN3_INCLUDE_DIR})
  add_definitions(-DAERO_HAVE_EIGEN)
endif()

# find ROS
find_package(catkin REQUIRED COMPONENTS roscpp pr2_controllers_msgs)
if(NOT catkin_INCLUDE_DIRS)
  message(WARNING "required packages not found")
else()
  message(STATUS "found required packages")
  include_directories(${catkin_INCLUDE_DIRS})
  add_definitions(-DAERO_HAVE_ROS)
endif()

catkin_package()

if (catkin_INCLUDE_DIRS)
  add_executable(aero_joint_state_publisher src/AeroJointStatePublisher.cc)
  target_link_libraries(aero_joint_state_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES})
endif()

# catkin build aero_world --verbose