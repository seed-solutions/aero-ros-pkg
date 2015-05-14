#if(NOT USE_ROSBUILD)
#  include(catkin.cmake)
#  return()
#endif()

cmake_minimum_required(VERSION 2.8.3)
project(aero_ros_bridge)

find_package(catkin REQUIRED COMPONENTS
  hrpsys_ros_bridge
)

compile_openhrp_model(${PROJECT_SOURCE_DIR}/models/aero-lower.wrl)
compile_openhrp_model(${PROJECT_SOURCE_DIR}/models/aero_upper.wrl)
execute_process(COMMAND bash ${PROJECT_SOURCE_DIR}/scripts/make_angle_vector_command.sh)
execute_process(COMMAND bash ${PROJECT_SOURCE_DIR}/scripts/make_stroke_vector_command.sh)
