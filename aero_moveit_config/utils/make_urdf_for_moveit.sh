#!/bin/bash

# prerequisites : aero_description/models/aero.urdf
# prerequisites : aero_moveit_config/models/aero_virtual_lifter.urdf

# generates : aero_description/models/aero_moveit.urdf

concatenate_urdf() {

    paste_to="$(rospack find aero_description)/models/aero.urdf"
    copy_from="$(rospack find aero_moveit_config)/models/aero_virtual_lifter.urdf"
    output="$(rospack find aero_description)/models/aero_moveit.urdf"

    names="$(rospack find aero_moveit_config)/models/aero_virtual_lifter.txt"

#    cp $paste_to $output
    cat $paste_to | sed -e 's/continuous/revolute/g' > $output

    cat ${names} | while read line
    do
        type=$(echo "$line" | cut -d ' ' -f1)
	name=$(echo "$line" | cut -d ' ' -f2)
	if [[ $type == "link" ]]
	then
	    has_name=$(grep $name $paste_to)
	    if [[ $has_name == "" ]]
	    then
		write_to=$(grep -n -m 1 "<joint" $output | cut -d ':' -f1)
		awk "/<link name=\"${name}\">/,/<\/link>/" $copy_from | sed -e 's/\//\\\//g; s/\"/\\\"/g' > /tmp/virtual_tmp_tmp
		sed '1!G;h;$!d' /tmp/virtual_tmp_tmp | xargs -I{} sed -i "${write_to}i\{}" $output
            fi
        elif [[ $type == "joint"  ]] # expects joints defined after links
        then
            has_name=$(grep $name $paste_to)
	    if [[ $has_name == "" ]]
            then
	        write_to=$(grep -n -m 1 "<\/robot" $output | cut -d ':' -f1)
		awk "/<joint name=\"${name}\"/,/<\/joint>/" $copy_from | sed -e 's/\//\\\//g; s/\"/\\\"/g' > /tmp/virtual_tmp_tmp
		sed '1!G;h;$!d' /tmp/virtual_tmp_tmp | xargs -I{} sed -i "${write_to}i\{}" $output
            fi
        fi
    done
}

replace_limits() {
    file=$1
    joint=$2
    lower=$3
    upper=$4
    line=$(cat ${file}| grep -n -A 10  ${joint} | grep -m 1 limit | cut -d '-' -f1)
    #echo "${1}: ${2} ${line}"

    lower_original=`echo "cat /robot/joint[@name=\"${joint}\"]/limit/@lower" | xmllint --shell ${file}  | grep "lower=" | sed 's/[^"]*"\([^"]*\)"[^"]*/\1/g'`
    upper_original=`echo "cat /robot/joint[@name=\"${joint}\"]/limit/@upper" | xmllint --shell ${file}  | grep "upper=" | sed 's/[^"]*"\([^"]*\)"[^"]*/\1/g'`

    sed -i "${line}s/lower=\"${lower_original}/lower=\"${lower}/" $file
    sed -i "${line}s/upper=\"${upper_original}/upper=\"${upper}/" $file
    #echo $upper
    #echo $upper_original
}

remove_mesh() {
    file=$1
    sed -i '/mesh/c\<!---->' $file
}
remove_geometry() {
    file=$1
    sed -i '/geometry/c\<!---->' $file
}

concatenate_urdf
original="$(rospack find aero_description)/models/aero_moveit.urdf"
file_mg="$(rospack find aero_description)/models/aero_moveit_limited.urdf"
file_ho="$(rospack find aero_description)/models/aero_moveit_limited_height_only.urdf"
file_op="$(rospack find aero_description)/models/aero_moveit_limited_on_plane.urdf"

cp $original $file_mg
cp $original $file_ho
cp $original $file_op

replace_limits $file_mg "l_wrist_p_joint" -0.04 0.04
replace_limits $file_mg "r_wrist_p_joint" -0.04 0.04
replace_limits $file_mg "l_wrist_y_joint" -1.5708 1.5708
replace_limits $file_mg "r_wrist_y_joint" -1.5708 1.5708
replace_limits $file_mg "l_shoulder_y_joint" -1.5708 1.5708
replace_limits $file_mg "r_shoulder_y_joint" -1.5708 1.5708
replace_limits $file_mg "waist_r_joint" -0.03 0.03
replace_limits $file_mg "waist_p_joint" 0.0 0.523599
replace_limits $file_mg "virtual_lifter_x_joint" -0.2 0.2
replace_limits $file_mg "virtual_lifter_z_joint" -0.4 0.0

replace_limits $file_ho "l_wrist_p_joint" -0.04 0.04
replace_limits $file_ho "r_wrist_p_joint" -0.04 0.04
replace_limits $file_ho "l_wrist_y_joint" -1.5708 1.5708
replace_limits $file_ho "r_wrist_y_joint" -1.5708 1.5708
replace_limits $file_ho "l_shoulder_y_joint" -1.5708 1.5708
replace_limits $file_ho "r_shoulder_y_joint" -1.5708 1.5708
replace_limits $file_ho "waist_r_joint" -0.03 0.03
replace_limits $file_ho "waist_p_joint" 0.0 0.523599
replace_limits $file_ho "virtual_lifter_x_joint" -0.0 0.0
replace_limits $file_ho "virtual_lifter_z_joint" -0.4 0.0

replace_limits $file_op "l_wrist_p_joint" -0.04 0.04
replace_limits $file_op "r_wrist_p_joint" -0.04 0.04
replace_limits $file_op "l_wrist_y_joint" -1.5708 1.5708
replace_limits $file_op "r_wrist_y_joint" -1.5708 1.5708
replace_limits $file_op "l_shoulder_y_joint" -1.5708 1.5708
replace_limits $file_op "r_shoulder_y_joint" -1.5708 1.5708
replace_limits $file_op "waist_r_joint" -0.03 0.03
replace_limits $file_op "waist_p_joint" 0.0 0.523599
replace_limits $file_op "virtual_lifter_x_joint" -0.2 0.2
replace_limits $file_op "virtual_lifter_z_joint" -0.3 -0.08

#remove_mesh $file_mg
#remove_mesh $file_ho
#remove_mesh $file_op

#remove_geometry $file_mg
#remove_geometry $file_ho
#remove_geometry $file_op
