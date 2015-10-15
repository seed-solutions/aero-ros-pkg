cmake_minimum_required(VERSION 2.8.3)
project(aero_description)

find_package(catkin REQUIRED COMPONENTS
  hrpsys_ros_bridge
)

# compile_openhrp_model(${PROJECT_SOURCE_DIR}/models/aero-lower.wrl)
# compile_openhrp_model(${PROJECT_SOURCE_DIR}/models/aero_upper.wrl)

catkin_package()