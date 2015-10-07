#!/bin/bash

remap_urdf_names() { # urdf_name
    txt="$(rospack find aero_ros_bridge)/config/${1}.txt"
    j=0
    urdf="$(rospack find aero_ros_bridge)/models/${1}.urdf"
    cp $urdf /tmp/$1_tmp
    while read line
    do
	type=$(echo "$line" | cut -d ' ' -f1)
	input_name=$(echo "$line" | cut -d ' ' -f2)
	output_name=$(echo "$line" | cut -d ' ' -f3)
	if [[ $type == "link" ]]
	then
	    sed -i "s/\"$input_name\"/\"$output_name\"/g" /tmp/$1_tmp # "" to prevent replacing filename
	elif [[ $type == "joint" ]]
	then
	    sed -i "s/$input_name/$output_name/g" /tmp/$1_tmp
	fi
	j=$(($j + 1))
    done < $txt
}

concatenate_urdf() { # paste_to copy_from output_file
    paste_to="/tmp/${1}_tmp"
    copy_from="/tmp/${2}_tmp"
    txt="$(rospack find aero_ros_bridge)/config/${2}.txt"
    output_file="$(rospack find aero_ros_bridge)/models/${3}.urdf"
    cp /tmp/$1_tmp $output_file
    while read line
    do
	type=$(echo "$line" | cut -d ' ' -f1)
	input_name=$(echo "$line" | cut -d ' ' -f2)
	output_name=$(echo "$line" | cut -d ' ' -f3)
	if [[ $type == "link" ]]
	then
	    has_name=$(grep $output_name $paste_to)
	    if [[ $has_name == "" ]]
	    then
		write_to=$(grep -n -m 1 "<joint" $paste_to | cut -d ':' -f1)
		awk "/<link name=\"${output_name}\">/,/<\/gazebo>/" $copy_from | sed -e 's/\//\\\//g; s/\"/\\\"/g' > /tmp/$1_tmp_tmp
		sed '1!G;h;$!d' /tmp/$1_tmp_tmp | xargs -I{} sed -i "${write_to}i\{}" $output_file
	    fi
	elif [[ $type == "joint"  ]] # expects joints defined after links
	then
	    has_name=$(grep $output_name $paste_to)
	    if [[ $has_name == "" ]]
	    then
		write_to=$(grep -n -m 1 "</robot" $output_file | cut -d ':' -f1)
		awk "/<joint name=\"${output_name}\"/,/<\/gazebo>/" $copy_from | sed -e 's/\//\\\//g; s/\"/\\\"/g' > /tmp/$1_tmp_tmp
		sed '1!G;h;$!d' /tmp/$1_tmp_tmp | xargs -I{} sed -i "${write_to}i\{}" $output_file
	    fi
	fi
    done < $txt
}

remap_urdf_names aero_upper
remap_urdf_names aero-lower
concatenate_urdf aero_upper aero-lower aero
