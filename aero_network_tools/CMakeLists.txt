cmake_minimum_required(VERSION 2.8.3)
project(aero_network_tools)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(FILES
  AeroNarrowCommand.msg
  AeroNarrowResponse.msg
  AeroJointStrokes.msg
  AeroJointAngles.msg
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)