#!/bin/bash

dir=$1
upper_name=$2
lower_name=$3

printf "Creating urdf ...\n"

./scripts/create_urdf.sh $dir $upper_name $lower_name

printf "\nwriting files to aero_startup/aero_hardware_interface ...\n"

./scripts/make_controller.sh $dir

printf "\nwriting files to aero_startup/aero_controller_manager ...\n"

./scripts/make_joint_state_publisher.sh $dir

printf "\nmaking Angle2Stroke.hh ... (takes about 20 sec)\n"

./scripts/make_angle_to_stroke_header.sh $dir $upper_name $lower_name

printf "\nmaking Stroke2Angle.hh ... (takes about 10 sec)\n"

./scripts/make_stroke_to_angle_header.sh $dir

printf "\nconfigurating controllers ...\n"

./scripts/configure_controllers.sh $dir
catkin b aero_startup

printf "\nfinished.\n"
