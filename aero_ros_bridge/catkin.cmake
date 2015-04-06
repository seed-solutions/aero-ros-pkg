cmake_minimum_required(VERSION 2.8.3)
project(aero_ros_bridge)

#find_package(catkin REQUIRED COMPONENTS hrpsys_ros_bridge hrpsys openhrp3)
find_package(catkin REQUIRED COMPONENTS hrpsys_ros_bridge)



compile_openhrp_model(${PROJECT_SOURCE_DIR}/models/aero.wrl)
#compile_openhrp_model(${PROJECT_SOURCE_DIR}/models/aero-stick-figure.wrl aero-stick-figure)

#compile_openhrp_model(${PROJECT_SOURCE_DIR}/models/sample1.wrl SampleRobot)

#compile_openhrp_model(
#  ${SAMPLE1_PATH} SampleRobot
#  --conf-file-option "abc_leg_offset: 0,0.09,0"
#  --conf-file-option "abc_stride_parameter: 0.15,0.05,10"
#  --conf-file-option "end_effectors: lleg,LLEG_ANKLE_R,WAIST,0.0,0.0,-0.07,0.0,0.0,0.0,0.0, rleg,RLEG_ANKLE_R,WAIST,0.0,0.0,-0.07,0.0,0.0,0.0,0.0, larm,LARM_WRIST_P,CHEST,0.0,0,-0.12,0,1.0,0.0,1.5708, rarm,RARM_WRIST_P,CHEST,0.0,0,-0.12,0,1.0,0.0,1.5708,"
#  --conf-file-option "collision_pair: RARM_WRIST_P:WAIST LARM_WRIST_P:WAIST RARM_WRIST_P:RLEG_HIP_R LARM_WRIST_P:LLEG_HIP_R RARM_WRIST_R:RLEG_HIP_R LARM_WRIST_R:LLEG_HIP_R"
#  --robothardware-conf-file-option "pdgains.file_name: ${PROJECT_SOURCE_DIR}/models/PDgains.sav"
#  )

