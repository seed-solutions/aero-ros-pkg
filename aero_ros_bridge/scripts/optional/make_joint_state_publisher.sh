#!/bin/bash

declare -a joint_number
joint_number=0
echo "${joint_number}" > /tmp/jointscc

create_joint_states() { # urdf_config
    code="\n"
    tab6=$'      '
    read -r -a joint_number < /tmp/jointscc

    while read line
    do
	type=$(echo "$line" | cut -d ' ' -f1)
	if [[ $type == "joint" ]]
	then
	    joint_name=$(echo "$line" | cut -d ' ' -f3)
	    code="${code}this->jointState.name[${joint_number}] = \"${joint_name}\";\n"
	    joint_number=$(($joint_number + 1))
	fi
    done < $(rospack find aero_ros_bridge)/config/${1}.txt

    echo "${joint_number}" > /tmp/jointscc
    echo -e "${code}"
}

tab6=$'      '
body=''
part=''

part="${part}$(create_joint_states aero_upper)"
part="${part}$(create_joint_states aero-lower)"

read -r -a joint_number < /tmp/jointscc

body="this->jointState.name.resize(${joint_number});\n"
body="${body}this->jointState.position.resize(${joint_number});\n${part}"
echo -e "${body}" > /tmp/mjointstpubcc

input_file="$(rospack find aero_ros_bridge)/templates/AeroJointStatePublisher.cc"
output_file="$(rospack find aero_ros_bridge)/../aero_world/src/AeroJointStatePublisher.cc"
cp $input_file $output_file
write_to_line=$(grep -n -m 1 "void AeroJointStatePublisher::Init()" $input_file | cut -d ':' -f1)
write_to_line=$(($write_to_line + 2))

sed -i '1!G;h;$!d' /tmp/mjointstpubcc

while read line
do
    echo -e "${tab6}${line}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $output_file
done < /tmp/mjointstpubcc
