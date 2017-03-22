cmake_minimum_required(VERSION 2.8.3)
project(aero_sensors)

find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS xtion/include kinect/include
  LIBRARIES aerosensors_xtion aerosensors_rotatekinectaddon
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
)

include_directories(
  xtion/include
  kinect/include
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-std=c++11)

# xtion library

add_library(aerosensors_xtion xtion/src/XtionInterface.cc)
add_library(aerosensors_rotatekinectaddon kinect/src/RotateKinectAddon.cc)


# examples

find_package(PCL 1.8)
if(NOT PCL_INCLUDE_DIRS)
  message(WARNING "PCL not found, disabled")
else()
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  set(FOUND_PCL 1)
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
  set(FOUND_OpenCV 1)
endif()

include(CheckIncludeFileCXX)
check_include_file_cxx(opencv2/saliency.hpp FOUND_OpenCV_CONTRIB)
if(NOT FOUND_OpenCV_CONTRIB)
  message(WARNING "opencv contrib not found, disabled")
else()
  message(STATUS "found opencv contrib")
endif()

# xtion examples

add_executable(points_sample
  xtion/samples/points.cc
  xtion/src/XtionInterface.cc
  )
target_link_libraries(points_sample
  ${catkin_LIBRARIES})

add_executable(points_compressed_sample
  xtion/samples/points_compressed.cc
  xtion/src/XtionInterface.cc
  )
target_link_libraries(points_compressed_sample
  ${catkin_LIBRARIES})

add_executable(image_sample
  xtion/samples/image.cc
  xtion/src/XtionInterface.cc
  )
target_link_libraries(image_sample
  ${catkin_LIBRARIES})

if (FOUND_OpenCV AND FOUND_OpenCV_CONTRIB)
  add_executable(image_centers_sample
    xtion/samples/image_centers.cc
    xtion/src/XtionInterface.cc
    )
  target_link_libraries(image_centers_sample
    ${catkin_LIBRARIES} ${OpenCV_LIBS})
endif()

if (FOUND_OpenCV AND FOUND_PCL)
  add_executable(image_bounds_sample
    xtion/samples/image_bounds.cc
    xtion/src/XtionInterface.cc
    )
  target_link_libraries(image_bounds_sample
    ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
endif()