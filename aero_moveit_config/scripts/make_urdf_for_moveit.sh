#!/bin/bash

# prerequisites : aero_description/models/aero.urdf
# prerequisites : aero_moveit_config/models/aero_virtual_lifter.urdf

# generates : aero_description/models/aero_moveit.urdf

concatenate_urdf() {

    paste_to="$(rospack find aero_description)/models/aero.urdf"
    copy_from="$(rospack find aero_moveit_config)/models/aero_virtual_lifter.urdf"
    output="$(rospack find aero_description)/models/aero_moveit.urdf"

    names="$(rospack find aero_moveit_config)/models/aero_virtual_lifter.txt"

    cp $paste_to $output
#    cat $paste_to | sed -e 's/continuous/revolute/g' > $output

    cat ${names} | while read line
    do
        type=$(echo "$line" | cut -d ' ' -f1)
	name=$(echo "$line" | cut -d ' ' -f2)
	if [[ $type == "link" ]]
	then
	    has_name=$(grep $name $paste_to)
	    if [[ $has_name == "" ]]
	    then
                echo "link"
		write_to=$(grep -n -m 1 "<joint" $paste_to | cut -d ':' -f1)
		awk "/<link name=\"${name}\">/,/<\/link>/" $copy_from | sed -e 's/\//\\\//g; s/\"/\\\"/g' > /tmp/virtual_tmp_tmp
		sed '1!G;h;$!d' /tmp/virtual_tmp_tmp | xargs -I{} sed -i "${write_to}i\{}" $output
            fi
        elif [[ $type == "joint"  ]] # expects joints defined after links
        then
            has_name=$(grep $name $paste_to)
	    if [[ $has_name == "" ]]
            then
                echo "joint"
                #echo $name
	        write_to=$(grep -n -m 1 "<\/robot" $paste_to | cut -d ':' -f1)
                #cat $copy_from
                awk "/<joint name=\"${name}\"/,/<\/joint>/" $copy_from
		awk "/<joint name=\"${name}\"/,/<\/joint>/" $copy_from | sed -e 's/\//\\\//g; s/\"/\\\"/g' > /tmp/virtual_tmp_tmp
                cat /tmp/virtual_tmp_tmp
		sed '1!G;h;$!d' /tmp/virtual_tmp_tmp | xargs -I{} sed -i "${write_to}i\{}" $output
            fi
        fi
    done
}
concatenate_urdf
