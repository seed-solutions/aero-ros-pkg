cmake_minimum_required(VERSION 2.8.3)
project(aero_addon)

find_package(catkin REQUIRED COMPONENTS
  aero_std
)

################## optional: ##################
find_package(PCL 1.8)
if(NOT PCL_INCLUDE_DIRS)
  message(WARNING "PCL not found, disabled")
else()
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
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
endif()

find_package(linux_kinect)
find_package(aero_sensors)

if(linux_kinect_FOUND)
  include_directories(${linux_kinect_INCLUDE_DIRS})
  link_directories(${linux_kinect_LIBRARIES})
endif()

if(aero_sensors_FOUND)
  include_directories(${aero_sensors_INCLUDE_DIRS})
  link_directories(${aero_sensors_LIBRARIES})
endif()

if (linux_kinect_FOUND AND aero_sensors_FOUND AND OpenCV_FOUND AND PCL_INCLUDE_DIRS)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES grasp_addon grasp_addon_extra
 CATKIN_DEPENDS aero_std
)
################## :optional ##################

else()
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES grasp_addon
 CATKIN_DEPENDS aero_std
)
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-std=c++11)

add_library(grasp_addon
  src/GraspAddon.cc
)
target_link_libraries(grasp_addon ${catkin_LIBRARIES})

################## optional: ##################
if (linux_kinect_FOUND AND aero_sensors_FOUND AND OpenCV_FOUND AND PCL_INCLUDE_DIRS)
add_library(grasp_addon_extra
  src/GraspAddonExtra.cc
)
target_link_libraries(grasp_addon_extra ${catkin_LIBRARIES})
endif()
################## :optional ##################
