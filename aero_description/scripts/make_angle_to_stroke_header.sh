#!/bin/bash

# prerequisites : {my_robot}/headers/Angle2Stroke.hh
# prerequisites : /tmp/aero_ros_order_upper < create_urdf
# prerequisites : /tmp/aero_ros_order_lower < create_urdf

# generates : aero_startup/aero_hardware_interface/Angle2Stroke.hh
# generates : aero_startup/aero_hardware_interface/Angle2Stroke.cc

dir=$1
upper_file=$2
lower_file=$3

input_file="$(rospack find aero_description)/${dir}/headers/Angle2Stroke.hh"
output_file="$(rospack find aero_description)/../aero_startup/aero_hardware_interface/Angle2Stroke.hh"
output_source="$(rospack find aero_description)/../aero_startup/aero_hardware_interface/Angle2Stroke.cc"
template_file="$(rospack find aero_description)/../aero_startup/.templates/aero_hardware_interface/Angle2Stroke.hh"

replace_meta_in_output_file() {
    output_file=$1

    total_joints=$(grep -o 'meta =' $output_file | wc -l)
    for (( joint_number=0; joint_number<=${total_joints}; joint_number++ ))
    do
	sed -i "0,/meta =/s//_strokes[${joint_number}] =/" $output_file
    done

    i=0    

    while read line
    do
	sed -i "s/${line}/_angles[${i}]/g" $output_file
	i=$(($i + 1))
    done < /tmp/aero_ros_order_upper

    while read line
    do
	sed -i "s/${line}/_angles[${i}]/g" $output_file
	i=$(($i + 1))
    done < /tmp/aero_ros_order_lower
}

create_table_func_from_csv() {
    joint_name=$1
    # offset=$2
    output_file=$3
    function_name=$4
    template_file=$5

    # parse joint_name
    file=''
    csv=$(echo "$joint_name" | awk -F/ '{print $2}')
    if [[ $csv == '' ]]
    then
	file="$(rospack find aero_description)/${dir}/models/csv/${joint_name}.csv"
    else
	csvdir=$(echo "$joint_name" | awk -F/ '{print $1}')
	file="$(rospack find aero_description)/${csvdir}/models/csv/${csv}.csv"
    fi

    # load csv
    table=()
    interval=()
    j=0
    while read line
    do  
        case_num[j]=$(echo "$line" | cut -d ',' -f1)
	table[j]=$(echo "$line" | cut -d ',' -f4)
	interval[j]=$(echo "$line" | cut -d ',' -f3)
	j=$(($j + 1))
    done < $file

    tab6=$'      '
    tab8=$'        '
    code=''

    idx=0
    for e in "${table[@]}"
    do
	val=$(echo "$2 + $e" | bc)
	code="${code}${tab6}case ${case_num[${idx}]}:\n"
	code="${code}${tab8}stroke = ${val};\n"
	code="${code}${tab8}interval = ${interval[${idx}]};\n"
	code="${code}${tab8}break;\n"
	idx=$(($idx + 1))
    done

    echo -e "${code}" > /tmp/mjointsstrokehhwrite

    awk "/float TableTemplate/,/};/" $template_file > /tmp/mjointsstrokehh
    sed -i "s/TableTemplate/${function_name}/g" /tmp/mjointsstrokehh

    write_to_line=$(grep -n -m 1 "void Angle2Stroke" $output_file | cut -d ':' -f1)
    write_to_line=$(($write_to_line - 1))
    write_to_top_line=$write_to_line

    IFS=''
    while read line
    do
	echo "${line}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $output_file
	write_to_line=$(($write_to_line + 1))
    done < /tmp/mjointsstrokehh

    write_to_line=$(grep -n -m 1 "switch (roundedAngle)" /tmp/mjointsstrokehh | cut -d ':' -f1)
    write_to_line=$(($write_to_line + 1))
    write_to_line=$(($write_to_top_line + $write_to_line))

    IFS=''
    while read line
    do
	echo "${line}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $output_file
	write_to_line=$(($write_to_line + 1))
    done < /tmp/mjointsstrokehhwrite

    sed -i "${write_to_top_line}i\    //////////////////////////////////////////////////" $output_file
    sed -i "${write_to_top_line}i\ " $output_file
}

create_rp_table_func_from_csv() {
    joint_name_a=$1
    joint_name_b=$2
    output_file=$3
    function_name=$4
    offset_p=$5
    offset_r=$6
    template_file=$7

    # parse joint_name_a
    file=''
    csv=$(echo "$joint_name_a" | awk -F/ '{print $2}')
    if [[ $csv == '' ]]
    then
	file="$(rospack find aero_description)/${dir}/models/csv/${joint_name_a}.csv"
    else
	csvdir=$(echo "$joint_name" | awk -F/ '{print $1}')
	file="$(rospack find aero_description)/${csvdir}/models/csv/${csv}.csv"
    fi

    # load roll csv
    case_num_r=()
    table_r=()
    interval_r=()
    j=0
    while read line
    do
	case_num_r[j]=$(echo "$line" | cut -d ',' -f1)
	table_r[j]=$(echo "$line" | cut -d ',' -f4)
	interval_r[j]=$(echo "$line" | cut -d ',' -f3)
	j=$(($j + 1))
    done < $file

    # parse joint_name_b
    file=''
    csv=$(echo "$joint_name_b" | awk -F/ '{print $2}')
    if [[ $csv == '' ]]
    then
	file="$(rospack find aero_description)/${dir}/models/csv/${joint_name_b}.csv"
    else
	csvdir=$(echo "$joint_name" | awk -F/ '{print $1}')
	file="$(rospack find aero_description)/${csvdir}/models/csv/${csv}.csv"
    fi

    # load pitch csv
    case_num_p=()
    table_p=()
    interval_p=()
    j=0
    while read line
    do
	case_num_p[j]=$(echo "$line" | cut -d ',' -f1)
	table_p[j]=$(echo "$line" | cut -d ',' -f4)
	interval_p[j]=$(echo "$line" | cut -d ',' -f3)
	j=$(($j + 1))
    done < $file

    tab6=$'      '
    tab8=$'        '
    code=''

    idx=0
    for e in "${table_p[@]}"
    do
	val_p=$(echo "$offset_p + $e" | bc)
	code="${code}${tab6}case ${case_num_p[${idx}]}:\n"
	code="${code}${tab8}stroke2 = ${val_p};\n"
	code="${code}${tab8}interval2 = ${interval[${idx}]};\n"
	code="${code}${tab8}break;\n"
	idx=$(($idx + 1))
    done

    echo -e "${code}" > /tmp/mjointsstrokehhwrite

    awk "/dualJoint TableTemplate/,/};/" $template_file > /tmp/mjointsstrokehh
    sed -i "s/TableTemplate/${function_name}/g" /tmp/mjointsstrokehh

    write_to_line=$(grep -n -m 1 "void Angle2Stroke" $output_file | cut -d ':' -f1)
    write_to_line=$(($write_to_line - 1))
    write_to_top_line=$write_to_line

    IFS=''
    while read line
    do
	echo "${line}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $output_file
	write_to_line=$(($write_to_line + 1))
    done < /tmp/mjointsstrokehh

    write_to_line=$(grep -n -m 1 "switch (roundedAngle2)" /tmp/mjointsstrokehh | cut -d ':' -f1)
    write_to_line=$(($write_to_line + 1))
    write_to_line=$(($write_to_top_line + $write_to_line))

    IFS=''
    while read line
    do
	echo "${line}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $output_file
	write_to_line=$(($write_to_line + 1))
    done < /tmp/mjointsstrokehhwrite

    idx=0
    code=''

    for e in "${table_r[@]}"
    do
	val_r=$(echo "$offset_r + $e" | bc)
	code="${code}${tab6}case ${case_num_r[${idx}]}:\n"
	code="${code}${tab8}stroke1 = ${val_r};\n"
	code="${code}${tab8}interval1 = ${interval[${idx}]};\n"
	code="${code}${tab8}break;\n"
	idx=$(($idx + 1))
    done

    echo -e "${code}" > /tmp/mjointsstrokehhwrite

    write_to_line=$(grep -n -m 1 "switch (roundedAngle1)" /tmp/mjointsstrokehh | cut -d ':' -f1)
    write_to_line=$(($write_to_line + 1))
    write_to_line=$(($write_to_top_line + $write_to_line))

    IFS=''
    while read line
    do
	echo "${line}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $output_file
	write_to_line=$(($write_to_line + 1))
    done < /tmp/mjointsstrokehhwrite

    sed -i "${write_to_top_line}i\    //////////////////////////////////////////////////" $output_file
    sed -i "${write_to_top_line}i\ " $output_file
}

cp $input_file $output_file
replace_meta_in_output_file $output_file

total_tables=$(grep -o '@define' $output_file | wc -l)
for (( table_number=1; table_number<=${total_tables}; table_number++ ))
do
    line=$(awk "/@define/{i++}i==${table_number}{print; exit}" $output_file)
    table_type=$(echo "${line}" | awk '{print $7}')
    func=$(echo "${line}" | awk '{print $2}')
    if [[ $table_type == '' ]]
    then
	csv=$(echo "${line}" | awk '{print $4}')
	offset=$(echo "${line}" | awk '{print $6}')
	create_table_func_from_csv $csv $offset $output_file $func $template_file
    else
	csv1=$(echo "${line}" | awk '{print $4}')
	csv2=$(echo "${line}" | awk '{print $5}')
	offset_p=$(echo "${line}" | awk '{print $7}')
	offset_r=$(echo "${line}" | awk '{print $9}')
	create_rp_table_func_from_csv $csv1 $csv2 $output_file $func $offset_p $offset_r $template_file
    fi
done

sed -i "1,$((${total_tables} + 2))d" $output_file

# write warnings

sed -i "1i\/*" $output_file
sed -i "2i\ * This file auto-generated from script. Do not Edit!" $output_file
sed -i "3i\ * Original : aero_startup/.templates/aero_hardware_interface/Angle2Stroke.hh" $output_file
sed -i "4i\ * Original : aero_description/{my_robot}/headers/Angle2Stroke.hh" $output_file
sed -i "5i\*/" $output_file

# divide header and source

cp $output_file $output_source
sed -i "/struct/,/};/ d" $output_source
sed -i "/#ifndef/d" $output_source
sed -i "/#define/d" $output_source
sed -i "/#endif/d" $output_source
sed -i "s/};/}/g" $output_source

edit_start=$(grep -n -m 1 "//////" $output_file | cut -d ':' -f1)
tail -n +$edit_start $output_file > /tmp/aero_modify_header
sed -i "/{/,/};/ d" /tmp/aero_modify_header
sed -i "s/)/);/g" /tmp/aero_modify_header
head -n $edit_start $output_file > /tmp/aero_modify_header_head
cat /tmp/aero_modify_header_head > $output_file
cat /tmp/aero_modify_header >> $output_file
sed -i "/\/\/\/\/\/\//d" $output_file
sed -i "/#include \"aero_hardware_interface\/Angle2Stroke.hh\"/d" $output_file
