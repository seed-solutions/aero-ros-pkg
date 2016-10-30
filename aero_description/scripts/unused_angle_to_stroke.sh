#!/bin/bash

# prerequisites : {my_robot}/headers/Angle2Stroke.hh
# prerequisites : /tmp/aero_ros_order_upper < create_urdf
# prerequisites : /tmp/aero_ros_order_lower < create_urdf

# generates : aero_startup/aero_hardware_interface/UnusedAngle2Stroke.hh

dir=$1

input_file="$(rospack find aero_description)/${dir}/headers/Angle2Stroke.hh"
output_file="$(rospack find aero_description)/../aero_startup/aero_hardware_interface/UnusedAngle2Stroke.hh"
template_file="$(rospack find aero_description)/../aero_startup/.templates/aero_hardware_interface/UnusedAngle2Stroke.hh"
tmp_file="/tmp/uua2s"
dual_joint_map="/tmp/uua2s_dualj"

# copy from template

cp $template_file $output_file

# grep "meta =" from Angle2Stroke.hh and copy to tmpfile

echo "warning: brackets \"[]\" are not allowed in \"meta =\" equation"
echo "warning: dualJoint variables should not be declared outside function Angle2Stroke"
echo "warning: dualJoint variable declaration should be minimal and not redundant"
echo "warning: dualJoint table function should only use each joint name once"

cp $input_file $tmp_file

# replace joint names to "_angles[]" in tmpfile

i=0
while read line
do
    sed -i "s/${line}/_angles[${i}]/g" $tmp_file
    i=$(($i + 1))
done < /tmp/aero_ros_order_upper

while read line
do
    sed -i "s/${line}/_angles[${i}]/g" $tmp_file
    i=$(($i + 1))
done < /tmp/aero_ros_order_lower

# create dual joint variable to angle number relations

> $dual_joint_map

total_dualjoints=$(grep -o 'dualJoint' $tmp_file | wc -l)
search_start=$(grep -n -m2 "dualJoint" $tmp_file | tail -1 | cut -d: -f1) # exclude declaration

for (( joint_number=2; joint_number<${total_dualjoints}; joint_number++ ))
do
    write=$(grep -m$joint_number "dualJoint" $tmp_file | tail -1 | awk '{print $2}')

    # search for "_angles" between i-th and (i+1)-th "dualJoint"
    search_end=$(grep -n -m$(($joint_number + 1)) "dualJoint" $tmp_file | tail -1 | cut -d: -f1)
    for (( line_number=$search_start; line_number<$search_end; line_number++ ))
    do
        line=$(sed "${line_number}q;d" $tmp_file | grep -o "_angles\[[0-9][0-9]\]")
        if [[ $line == "" ]]
        then
            line=$(sed "${line_number}q;d" $tmp_file | grep -o "_angles\[[0-9]\]")
            if [[ $line == "" ]]
            then
                continue
            fi
        fi
        write="$write $line"
    done
    echo $write >> $dual_joint_map
    search_start=$search_end
done

# joint_number = total_joints is exceptional

write=$(grep "dualJoint" $tmp_file | tail -1 | awk '{print $2}')

search_end=$(grep -n -m1 "meta =" $tmp_file | cut -d: -f1)
for (( line_number=$search_start; line_number<$search_end; line_number++ ))
do
    line=$(sed "${line_number}q;d" $tmp_file | grep -o "_angles\[[0-9][0-9]\]")
    if [[ $line == "" ]]
    then
        line=$(sed "${line_number}q;d" $tmp_file | grep -o "_angles\[[0-9]\]")
        if [[ $line == "" ]]
        then
            continue
        fi
    fi
    write="$write $line"
done
echo $write >> $dual_joint_map

# find relations between angle number and stroke number

tab6=$'      '
write_to_line=$(grep -n "implement here" $template_file | cut -d: -f1)

total_joints=$(grep -o 'meta =' $tmp_file | wc -l)
search_start=$(grep -n -m1 "meta =" $tmp_file | cut -d: -f1)
for (( joint_number=1; joint_number<${total_joints}; joint_number++ ))
do
    found_angle_num=""
    # search for "_angles" between i-th and (i+1)-th "meta ="
    search_end=$(grep -n -m$(($joint_number + 1)) "meta =" $tmp_file | tail -1 | cut -d: -f1)
    for (( search_number=$search_start; search_number<$search_end; search_number++ ))
    do
        line=$(sed "${search_number}q;d" $tmp_file)
        if [[ $line == *"_angles"* ]]
        then
            found_angle_num=$(echo $line | cut -d[ -f2 | cut -d] -f1)
            echo "${tab6}if (!_angles[$found_angle_num]) _strokes[$(($joint_number - 1))] = 0x7fff;" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $output_file
            break
        fi
    done
    # search for "_angles" from dual joint when "_angles" not found from equation
    if [[ $found_angle_num == "" ]]
    then
        while read line
        do
            dual_joint_name=$(echo $line | awk '{print $1}')
            for (( search_number=$search_start; search_number<=$search_end; search_number++ ))
            do
                l=$(sed "${search_number}q;d" $tmp_file | grep -o $dual_joint_name)
                if [[ $l != ""  ]]
                then
                    found_angle_num="true"
                    found_angle_num1=$(echo $line | awk '{print $2}')
                    found_angle_num2=$(echo $line | awk '{print $3}')
                    echo "${tab6}if (!$found_angle_num1 && !$found_angle_num2) _strokes[$(($joint_number - 1))] = 0x7fff;" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $output_file
                    break
                fi
            done
            if [[ $found_angle_num != "" ]]
            then
                break
            fi
        done < $dual_joint_map
    fi
    search_start=$search_end
done

# joint_number = total_joints is exceptional

search_end=$(grep -n "};" $tmp_file | tail -1 | cut -d: -f1)
for (( search_number=$search_start; search_number<$search_end; search_number++ ))
do
    line=$(sed "${search_number}q;d" $tmp_file)
    if [[ $line == *"_angles"* ]]
    then
        found_angle_num=$(echo $line | cut -d[ -f2 | cut -d] -f1)
        echo "${tab6}if (!_angles[$found_angle_num]) _strokes[$(($joint_number - 1))] = 0x7fff;" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $output_file
        break
    fi
done
# search for "_angles" from dual joint when "_angles" not found from equation
if [[ $found_angle_num == "" ]]
then
    while read line
    do
        dual_joint_name=$(echo $line | awk '{print $1}')
        for (( search_number=$search_start; search_number<=$search_end; search_number++ ))
        do
            l=$(sed "${search_number}q;d" $tmp_file | grep -o $dual_joint_name)
            if [[ $l != ""  ]]
            then
                found_angle_num="true"
                found_angle_num1=$(echo $line | awk '{print $2}')
                found_angle_num2=$(echo $line | awk '{print $3}')
                echo "${tab6}if (!$found_angle_num1 && !$found_angle_num2) _strokes[$(($joint_number - 1))] = 0x7fff;" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $output_file
                break
            fi
        done
        if [[ $found_angle_num != "" ]]
        then
            break
        fi
    done < $dual_joint_map
fi


# write warnings

sed -i "1i\/*" $output_file
sed -i "2i\ * This file auto-generated from script. Do not Edit!" $output_file
sed -i "3i\ * Original : aero_startup/.templates/aero_hardware_interface/UnusedAngle2Stroke.hh" $output_file
sed -i "4i\ * Original : aero_description/{my_robot}/headers/Angle2Stroke.hh" $output_file
sed -i "5i\*/" $output_file
