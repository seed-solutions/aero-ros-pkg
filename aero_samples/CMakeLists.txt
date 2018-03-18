cmake_minimum_required(VERSION 2.8.3)
project(aero_samples)

find_package(catkin REQUIRED COMPONENTS
  aero_std
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_definitions(-std=c++11)

# check ros_distro
if("$ENV{ROS_DISTRO}" STRGREATER "jade")
  add_definitions(-DKINETIC)
endif()

# samples

add_executable(minimum_sample_node src/minimum.cc)
target_link_libraries(minimum_sample_node ${catkin_LIBRARIES})

add_executable(fk_sample_node src/fk.cc)
target_link_libraries(fk_sample_node ${catkin_LIBRARIES})

add_executable(ik_sample_node src/ik.cc)
target_link_libraries(ik_sample_node ${catkin_LIBRARIES})

add_executable(interpolation_sample_node src/interpolation.cc)
target_link_libraries(interpolation_sample_node ${catkin_LIBRARIES})

add_executable(hand_sample_node src/hand.cc)
target_link_libraries(hand_sample_node ${catkin_LIBRARIES})

add_executable(look_at_sample_node src/look_at.cc)
target_link_libraries(look_at_sample_node ${catkin_LIBRARIES})

add_executable(look_at_external_sample_node src/look_at_external.cc)
target_link_libraries(look_at_external_sample_node ${catkin_LIBRARIES})

add_executable(look_at_topic_sample_node src/look_at_topic.cc)
target_link_libraries(look_at_topic_sample_node ${catkin_LIBRARIES})

add_executable(async_neck_control_sample_node src/async_neck_control.cc)
target_link_libraries(async_neck_control_sample_node ${catkin_LIBRARIES})

add_executable(top_grasp_sample_node src/top_grasp.cc)
target_link_libraries(top_grasp_sample_node ${catkin_LIBRARIES})

add_executable(side_grasp_sample_node src/side_grasp.cc)
target_link_libraries(side_grasp_sample_node ${catkin_LIBRARIES})

##add_executable(trajectory_lifter_sample_node src/trajectory_lifter.cc)
##target_link_libraries(trajectory_lifter_sample_node ${catkin_LIBRARIES})

add_executable(go_pos_sample_node src/go_pos.cc)
target_link_libraries(go_pos_sample_node ${catkin_LIBRARIES})

add_executable(async_sample_node src/async.cc)
target_link_libraries(async_sample_node ${catkin_LIBRARIES})

add_executable(async_lifter_cancel_node src/async_lifter_cancel.cc)
target_link_libraries(async_lifter_cancel_node ${catkin_LIBRARIES})

add_executable(set_marker_sample_node src/set_marker.cc)
target_link_libraries(set_marker_sample_node ${catkin_LIBRARIES})

add_executable(convert_world_sample_node src/convert_world.cc)
target_link_libraries(convert_world_sample_node ${catkin_LIBRARIES})

# tests

add_executable(stoa_atos_test_node tests/stoa_atos.cc)
target_link_libraries(stoa_atos_test_node ${catkin_LIBRARIES})

add_executable(fullbody_test_node tests/fullbody_test.cc)
target_link_libraries(fullbody_test_node ${catkin_LIBRARIES})

add_executable(lifter_ik_test_node tests/lifter_ik.cc)
target_link_libraries(lifter_ik_test_node ${catkin_LIBRARIES})

##add_executable(upper_controller_test_node tests/upper_controller_test.cc)
##target_link_libraries(upper_controller_test_node ${catkin_LIBRARIES})
