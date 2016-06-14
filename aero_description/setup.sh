#!/bin/bash

if [[ $1 = "--local" ]]
then
    dir=$2
    if [[ $dir = "" ]]
    then
	echo "error: usage[./setup.sh --local robot_dir]"
	exit 1
    else
	echo "setup local $dir"
    fi
else
    dir=$1
    if [[ $dir = "" ]]
    then
	echo "error: no robot specified"
	exit 1
    else
	echo "setup robot $dir"
    fi
fi

upper_file=$(find ./$dir -name "*upper*.txt" | awk -F "/" '{print $NF}')
if [[ $upper_file = "" ]]
then
    echo "error: make sure robot directory exists"
    echo "error: upper file must match pattern *upper*.txt"
    exit 1
fi

upper_name=$(echo "$upper_file" | awk -F "." '{print $1}')
lower_name=$(find ./$dir -name "*.txt" ! -name $upper_file | awk -F "/" '{print $NF}' | awk -F "." '{print $1}')

check_install=$(rospack find aero_description)
if [[ $check_install = "" ]]
then
    echo "error: aero_description must be built before running script"
    exit 1
fi

printf "Setting up models directory ...\n"

./scripts/setup_models_directory.sh $dir $upper_name $lower_name

printf "\nCreating urdf ...\n"

./scripts/create_urdf.sh $dir $upper_name $lower_name

if [[ $1 = "--local" ]]
then
    printf "\nsetting up srvs ...\n"

    ./scripts/install_srv.sh
else
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
fi

catkin b aero_startup

printf "\nfinished.\n"
