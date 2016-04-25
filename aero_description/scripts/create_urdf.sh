#!/bin/bash

# prerequisites : {my_robot}/{my_upper_body}.txt
# prerequisites : {my_robot}/{my_lower_body}.txt

# generates : aero_description/models/aero.urdf

dir=$1
upper_file=$2
lower_file=$3

remap_urdf_names() {
    urdf_name=$1
    save_tmp_to=$2

    txt="$(rospack find aero_description)/${dir}/${urdf_name}.txt"
    urdf="$(rospack find aero_description)/models/${urdf_name}.urdf"

    cp $urdf /tmp/$1_tmp

    > $save_tmp_to # empty file if file exists

    j=0
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
	    echo "${output_name}" >> $save_tmp_to
	fi
	j=$(($j + 1))
    done < $txt
}

concatenate_urdf() { # paste_to copy_from output_file
    paste_to="/tmp/${1}_tmp"
    copy_from="/tmp/${2}_tmp"
    txt="$(rospack find aero_description)/${dir}/${2}.txt"
    output_file="$(rospack find aero_description)/models/${3}.urdf"

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

remap_urdf_names $upper_file /tmp/aero_ros_order_upper
remap_urdf_names $lower_file /tmp/aero_ros_order_lower
concatenate_urdf $upper_file $lower_file aero
