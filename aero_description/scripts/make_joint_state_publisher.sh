#!/bin/bash

# prerequisites : /tmp/aero_ros_order_upper < create_urdf
# prerequisites : /tmp/aero_ros_order_lower < create_urdf

# generates : aero_startup/aero_controller_manager/AeroJointStatePublisher.cc

input_file="$(rospack find aero_description)/../aero_startup/.templates/aero_controller_manager/AeroJointStatePublisher.cc"
output_file="$(rospack find aero_description)/../aero_startup/aero_controller_manager/AeroJointStatePublisher.cc"

declare -a joint_number
joint_number=0
echo "${joint_number}" > /tmp/jointscc

cp $input_file $output_file

create_init() {
    file_read=$1
    read -r -a joint_number < /tmp/jointscc

    code="\n"

    while read line
    do
	code="${code}joint_state_.name[${joint_number}] = \"${line}\";\n"
	joint_number=$(($joint_number + 1))
    done < $file_read

    echo "${joint_number}" > /tmp/jointscc
    echo -e "${code}"
}

tab2=$'  '
body=''
part=''

part="${part}$(create_init /tmp/aero_ros_order_upper)"
part="${part}$(create_init /tmp/aero_ros_order_lower)"

read -r -a joint_number < /tmp/jointscc

body="joint_state_.name.resize(${joint_number});\n"
body="${body}joint_state_.position.resize(${joint_number});\n${part}"
echo -e "${body}" > /tmp/mjointstpubcc

write_to_line=$(grep -n -m 1 "void AeroJointStatePublisher::Init()" $output_file | cut -d ':' -f1)
write_to_line=$(($write_to_line + 2))

while read line
do
    echo -e "${tab2}${line}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $output_file
    write_to_line=$(($write_to_line + 1))
done < /tmp/mjointstpubcc

# create callback

body=''
for (( joint=0; joint<${joint_number}; joint++ ))
do
    body="${body}joint_state_.position[${joint}] = _msg.actual.positions[${joint}];\n"
done
echo -e "${body}" > /tmp/mjointstpubcc

write_to_line=$(grep -n -m 1 "void AeroJointStatePublisher::JointStateCallback" $output_file | cut -d ':' -f1)
write_to_line=$(($write_to_line + 3))

while read line
do
    echo -e "${tab2}${line}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $output_file
    write_to_line=$(($write_to_line + 1))
done < /tmp/mjointstpubcc

# write warnings

sed -i "1i\/*" $output_file
sed -i "2i\ * This file auto-generated from script. Do not Edit!" $output_file
sed -i "3i\ * Original : aero_startup/.templates/aero_controller_manager/AeroJointStatePublisher.cc" $output_file
sed -i "4i\ * Depend : aero_description/{my_robot}/{my_upper_file}.txt" $output_file
sed -i "5i\ * Depend : aero_description/{my_robot}/{my_lower_file}.txt" $output_file
sed -i "6i\*/" $output_file
