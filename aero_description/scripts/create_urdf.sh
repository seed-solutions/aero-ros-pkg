#!/bin/bash

# prerequisites : {my_robot}/{my_upper_body}.txt
# prerequisites : {my_robot}/{my_lower_body}.txt
# prerequisites : {my_robot}/robot.cfg

# generates : aero_description/urdf
# generates : aero_description/meshes
# generates : aero_description/robots

dir=$1
upper_file=$(rospack find aero_description)/${dir}/$2.txt
lower_file=$(rospack find aero_description)/${dir}/$3.txt
robot_file=$(rospack find aero_description)/${dir}/robot.cfg
aero_description_path=$(rospack find aero_description)

# these lines are needed from other scripts
cp ${upper_file} /tmp/aero_ros_order_upper
cp ${lower_file} /tmp/aero_ros_order_lower

# remove old directories : urdf, robots, meshes
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

# create directories
mkdir ${aero_description_path}/urdf/
mkdir ${aero_description_path}/meshes/
mkdir ${aero_description_path}/robots/

# check robot directories resources
if [ -e ${aero_description_path}/${dir}/urdf ]
then
    cp -r ${aero_description_path}/${dir}/urdf/* ${aero_description_path}/urdf/
fi
if [ -e ${aero_description_path}/${dir}/meshes ]
then
    cp -r ${aero_description_path}/${dir}/meshes/* ${aero_description_path}/meshes/
fi
if [ -e ${aero_description_path}/${dir}/robots ]
then
    cp -r ${aero_description_path}/${dir}/robots/* ${aero_description_path}/robots/
fi

# parse robot.cfg
while read line
do
    # distinguish commentout or parts name
    proto=$(echo $line | awk '{print $1}')
    if [[ $proto == "#" ]] # comment out
    then
        continue
    elif [[ $proto == ":" ]] # body name
    then
        continue
    fi

    # shop_dir: aero_shop or your pkg path
    # parts_dir: this must be directry under the ${shop_dir}
    shop_dir=$(echo $line | awk '{print $1}' | cut -d/ -f1)
    parts_dir=$(echo $line | awk '{print $1}' | cut -d/ -f2)
    if [[ $shop_dir == "aero_shop" ]]
    then # use relative path
        parts_path="$(rospack find aero_description)/../aero_shop/${parts_dir}"
    else
        parts_path=$(rospack find ${shop_dir})/${parts_dir}
    fi

    # check if parts has urdf directory
    if [ -e ${parts_path}/urdf ]
    then
        mkdir ${aero_description_path}/urdf/${parts_dir}
        cp -r ${parts_path}/urdf/* ${aero_description_path}/urdf/${parts_dir}
    fi

    # check if parts has meshes directory
    if [ -e ${parts_path}/meshes ]
    then
        if [ -e ${parts_path}/meshes/meshes.txt ]
        then
            # handle referenced mesh directory
            while read meshdir
            do
                meshshop_dir=$(echo $meshdir | cut -d/ -f1)
                meshparts_dir=$(echo $meshdir | cut -d/ -f2)
                if [[ $meshshop_dir == "aero_shop" ]]
                then
                    meshparts_path="$(rospack find aero_description)/../aero_shop/${meshparts_dir}"
                else
                    meshparts_path=$(rospack find ${meshshop_dir})/${meshparts_dir}
                fi
                mkdir ${aero_description_path}/meshes/${meshparts_dir}
                cp -r ${meshparts_path}/meshes/* ${aero_description_path}/meshes/${meshparts_dir}
            done < ${parts_path}/meshes/meshes.txt
        else
            mkdir ${aero_description_path}/meshes/${parts_dir}
            cp -r ${parts_path}/meshes/* ${aero_description_path}/meshes/${parts_dir}
        fi
    fi

done < $robot_file
