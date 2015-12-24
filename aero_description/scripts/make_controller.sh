#!/bin/bash

# prerequisites : {my_robot}/headers/Constants.hh
# prerequisites : /tmp/aero_ros_order_upper < create_urdf
# prerequisites : /tmp/aero_ros_order_lower < create_urdf

# generates : aero_startup/aero_hardware_interface/AeroControllers.cc
# generates : aero_startup/aero_hardware_interface/AngleJointNames.hh
# generates : aero_startup/aero_hardware_interface/AngleJointNames.cc
# generates : aero_startup/aero_hardware_interface/Constants.hh

dir=$1

input_header="$(rospack find aero_description)/${dir}/headers/Constants.hh"
output_header="$(rospack find aero_description)/../aero_startup/aero_hardware_interface/Constants.hh"

input_file="$(rospack find aero_description)/../aero_startup/.templates/aero_hardware_interface/AeroControllers.cc"
output_file="$(rospack find aero_description)/../aero_startup/aero_hardware_interface/AeroControllers.cc"

read_include="$(rospack find aero_description)/../aero_startup/.templates/aero_hardware_interface/AngleJointNames.hh"
generate_include="$(rospack find aero_description)/../aero_startup/aero_hardware_interface/AngleJointNames.hh"
generate_source="$(rospack find aero_description)/../aero_startup/aero_hardware_interface/AngleJointNames.cc"


grep "CAN" $input_header | awk '{print tolower($4)}' > /tmp/aero_CAN_order

cp $input_file $output_file

write_joint_indices() {
    id_tag=$1
    dof_tag=$2
    input_header=$3
    id_offset=$4
    id=$(grep "${id_tag}" $input_header | awk '{print $6}' | cut -d ';' -f1)
    num_of_dofs=$(grep "${dof_tag}" $input_header | awk '{print $6}' | cut -d ';' -f1)
    num_of_dofs=$(($num_of_dofs + $id_offset))
    start_joint=$(($id_offset + 1))

    tab2=$'  '
    tab6=$'      '
    code=''

    for (( joint_number=${start_joint}; joint_number<=${num_of_dofs}; joint_number++ ))
    do
	can_name=$(awk "/CAN/{i++}i==${joint_number}{print; exit}" $input_header | awk '{print $4}')
	can_id=$(awk "/CAN/{i++}i==${joint_number}{print; exit}" $input_header | awk '{print $6}' | cut -d ';' -f1)
	raw_id=$(grep $(echo ${can_name#CAN_}) $input_header | grep -E "RAW" | awk '{print $6}' | cut -d ';' -f1)
	offset=$(grep $(echo ${can_name#CAN_}) $input_header | grep -E "OFFSET" | awk '{print $6}' | cut -d ';' -f1)
	can_name=$(echo $can_name | awk '{print tolower($1)}')
	code="${code}${tab2}stroke_joint_indices_.push_back(\n"
	code="${code}${tab6}AJointIndex(${id}, ${can_id}, ${raw_id}, std::string(\"${can_name}\")));\n"
	if [[ $offset != '' ]]
	then
	    code="${code}${tab2}stroke_ref_vector_[${can_id}] = ${offset};\n"
	fi
    done

    echo -e "${code}" > /tmp/aero_hardware_interface_tmp_code_block
}

write_wheel_indices() {
    input_header=$1
    id=$(grep "ID_LOWER" $input_header | awk '{print $6}' | cut -d ';' -f1)
    num_of_dofs=$(grep "AERO_DOF_WHEEL" $input_header | awk '{print $6}' | cut -d ';' -f1)

    tab2=$'  '
    tab6=$'      '
    code=''

    for (( joint_number=1; joint_number<=${num_of_dofs}; joint_number++ ))
    do
	can_name=$(awk "/CAN/ && /WHEEL/{i++}i==${joint_number}{print; exit}" $input_header | awk '{print $4}')
	can_id=$(awk "/CAN/ && /WHEEL/{i++}i==${joint_number}{print; exit}" $input_header | awk '{print $6}' | cut -d ';' -f1)
	raw_id=$(grep $(echo ${can_name#CAN_}) $input_header | grep -E "RAW" | awk '{print $6}' | cut -d ';' -f1)
	can_name=$(echo $can_name | awk '{print tolower($1)}')
	code="${code}${tab2}wheel_indices_.push_back(\n"
	code="${code}${tab6}AJointIndex(${id}, ${can_id}, ${raw_id}, std::string(\"${can_name}\")));\n"
    done

    echo -e "${code}" >> /tmp/aero_hardware_interface_tmp_code_block
}

# write_angle_joint_indices also generates code for AngleJointNames.hh
write_angle_joint_indices() {
    code=''
    code_upper=''
    code_lower=''

    tab2=$'  '
    tab6=$'      '

    i=0

    while read line
    do
	code="${code}${tab6}_names[${i}] = \"${line}\";\n"
	code_upper="${code_upper}${tab2}angle_joint_indices_[\"${line}\"] = ${i};\n"
	i=$(($i + 1))
    done < /tmp/aero_ros_order_upper

    while read line
    do
	code="${code}${tab6}_names[${i}] = \"${line}\";\n"
	code_lower="${code_lower}${tab2}angle_joint_indices_[\"${line}\"] = ${i};\n"
	i=$(($i + 1))
    done < /tmp/aero_ros_order_lower

    echo -e "${code}" > /tmp/aero_angle_joint_names_tmp_code_block
    echo -e "${code_upper}" > /tmp/aero_hardware_interface_tmp_code_block_u
    echo -e "${code_lower}" > /tmp/aero_hardware_interface_tmp_code_block_l
}


# generate Constants.hh

cp $input_header $output_header

sed -i "1i\/*" $output_header
sed -i "2i\ * This file auto-generated from script. Do not Edit!" $output_header
sed -i "3i\ * Original : aero_description/{my_robot}/headers/Constants.hh" $output_header
sed -i "4i\*/" $output_header

# generate AngleJointNames.hh

cp $read_include $generate_include
write_to_line=$(grep -n -m 1 "};" $generate_include | cut -d ':' -f1)
write_angle_joint_indices

IFS=''
while read line
do
    echo "${line}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $generate_include
    write_to_line=$(($write_to_line + 1))
done < /tmp/aero_angle_joint_names_tmp_code_block

sed -i "1i\/*" $generate_include
sed -i "2i\ * This file auto-generated from script. Do not Edit!" $generate_include
sed -i "3i\ * Original : aero_startup/.templates/aero_hardware_interface/AngleJointNames.hh" $generate_include
sed -i "4i\ * Depend : aero_description/{my_robot}/{my_upper_file}.txt" $generate_include
sed -i "5i\ * Depend : aero_description/{my_robot}/{my_lower_file}.txt" $generate_include
sed -i "6i\*/" $generate_include

cp $generate_include $generate_source
sed -i "/#ifndef/d" $generate_source
sed -i "/#define/d" $generate_source
sed -i "/#endif/d" $generate_source
sed -i "s/};/}/g" $generate_source

edit_start=$(grep -n -m 1 "void" $generate_include | cut -d ':' -f1)
tail -n +$edit_start $generate_include > /tmp/aero_modify_header
sed -i "/{/,/};/ d" /tmp/aero_modify_header
sed -i "s/)/);/g" /tmp/aero_modify_header
head -n $(($edit_start - 1)) $generate_include > /tmp/aero_modify_header_head
cat /tmp/aero_modify_header_head > $generate_include
cat /tmp/aero_modify_header >> $generate_include

# complete AeroUpperController

awk "/AeroUpperController::AeroUpperController/,/}/" $output_file > /tmp/aero_hardware_interface_tmp
write_to_line=$(grep -n -m 1 "// adding code" /tmp/aero_hardware_interface_tmp | cut -d ':' -f1)
write_origin=$(grep -n -m 1 "AeroUpperController::AeroUpperController" $output_file | cut -d ':' -f1)
write_to_line=$(($write_origin + $write_to_line))

write_joint_indices "ID_UPPER" "AERO_DOF_UPPER" $input_header 0

IFS=''
while read line
do
    echo "${line}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $output_file
    write_to_line=$(($write_to_line + 1))
done < /tmp/aero_hardware_interface_tmp_code_block

IFS=''
while read line
do
    echo "${line}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $output_file
    write_to_line=$(($write_to_line + 1))
done < /tmp/aero_hardware_interface_tmp_code_block_u

# complete AeroLowerController

awk "/AeroLowerController::AeroLowerController/,/}/" $output_file > /tmp/aero_hardware_interface_tmp
write_to_line=$(grep -n -m 1 "// adding code" /tmp/aero_hardware_interface_tmp | cut -d ':' -f1)
write_origin=$(grep -n -m 1 "AeroLowerController::AeroLowerController" $output_file | cut -d ':' -f1)
write_to_line=$(($write_origin + $write_to_line))

lower_offset=$(grep "AERO_DOF_UPPER" $input_header | awk '{print $6}' | cut -d ';' -f1)
write_joint_indices "ID_LOWER" "AERO_DOF_LOWER" $input_header $lower_offset 
write_wheel_indices $input_header

IFS=''
while read line
do
    echo "${line}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $output_file
    write_to_line=$(($write_to_line + 1))
done < /tmp/aero_hardware_interface_tmp_code_block

IFS=''
while read line
do
    echo "${line}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $output_file
    write_to_line=$(($write_to_line + 1))
done < /tmp/aero_hardware_interface_tmp_code_block_l

write_encode_short() {
    input_header=$1

    tab2=$'  '
    tab6=$'      '
    code=''

    total_slaves=$(grep -o "NA"  $input_header | wc -l)
    for (( joint_number=1; joint_number<=${total_slaves}; joint_number++ ))
    do
	raw_id=$(awk "/NA/{i++}i==${joint_number}{print; exit}" $input_header | awk '{print $6}' | cut -d ';' -f1)
	code="${code}${tab2}encode_short_(_d0, &dat[RAW_HEADER_OFFSET + ${raw_id} * 2]);\n"
    done

    total_wheels=$(grep "AERO_DOF_WHEEL" $input_header | awk '{print $6}' | cut -d ';' -f1)
    for (( joint_number=1; joint_number<=${total_wheels}; joint_number++ ))
    do
	wheel_id=$(awk "/RAW/ && /WHEEL/{i++}i==${joint_number}{print; exit}" $input_header | awk '{print $6}' | cut -d ';' -f1)
	code="${code}${tab2}encode_short_(_d1, &dat[RAW_HEADER_OFFSET + ${wheel_id} * 2]);\n"
    done

    echo -e "${code}" > /tmp/aero_hardware_interface_tmp_code_block
}

awk "/AeroLowerController::servo_command/,/}/" $output_file > /tmp/aero_hardware_interface_tmp
write_to_line=$(grep -n -m 1 "// adding code" /tmp/aero_hardware_interface_tmp | cut -d ':' -f1)
write_origin=$(grep -n -m 1 "AeroLowerController::servo_command" $output_file | cut -d ':' -f1)
write_to_line=$(($write_origin + $write_to_line))

write_encode_short $input_header

IFS=''
while read line
do
    echo "${line}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $output_file
    write_to_line=$(($write_to_line + 1))
done < /tmp/aero_hardware_interface_tmp_code_block

# write warnings

sed -i "1i\/*" $output_file
sed -i "2i\ * This file auto-generated from script. Do not Edit!" $output_file
sed -i "3i\ * Original : aero_startup/.templates/aero_hardware_interface/AeroControllers.cc" $output_file
sed -i "4i\ * Depend : aero_description/{my_robot}/headers/Constants.hh" $output_file
sed -i "5i\*/" $output_file
