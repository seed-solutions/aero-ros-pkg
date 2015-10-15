#!/bin/bash

dir=$1

upper_file=$(find ./$dir -name "*upper*.txt" | awk -F "/" '{print $NF}')
upper_name=$(echo "$upper_file" | awk -F "." '{print $1}')
lower_name=$(find ./$dir -name "*.txt" ! -name $upper_file | awk -F "/" '{print $NF}' | awk -F "." '{print $1}')

printf "Creating urdf ...\n\n"

./scripts/create_urdf.sh $dir $upper_name $lower_name

printf "writing files to aero_startup/aero_controller ...\n\n"

./scripts/make_controller.sh $dir

printf "making Angle2Stroke.hh ... (takes about 20 sec)\n\n"

./scripts/make_angle_to_stroke_header.sh $dir $upper_name $lower_name

printf "making Stroke2Angle.hh ... (takes about 10 sec)\n\n"

./scripts/make_stroke_to_angle_header.sh $dir

printf "finished.\n"
