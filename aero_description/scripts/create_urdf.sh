#!/bin/bash

# prerequisites : {my_robot}/{my_upper_body}.txt
# prerequisites : {my_robot}/{my_lower_body}.txt

# generates : aero_description/models/aero.urdf

dir=$1
upper_file=$(rospack find aero_description)/${dir}/$2.txt
lower_file=$(rospack find aero_description)/${dir}/$3.txt

cp ${upper_file} /tmp/aero_ros_order_upper
cp ${lower_file} /tmp/aero_ros_order_lower

if [[ -d $(rospack find aero_description)/urdf ]]
then
    rm -rf $(rospack find aero_description)/urdf
fi

if [[ -d $(rospack find aero_description)/robots ]]
then
    rm -rf $(rospack find aero_description)/robots
fi

if [[ -d $(rospack find aero_description)/meshes ]]
then
    rm -rf $(rospack find aero_description)/meshes
fi

cp -r ${dir}/urdf $(rospack find aero_description)
cp -r ${dir}/robots $(rospack find aero_description)
cp -r ${dir}/meshes $(rospack find aero_description)
