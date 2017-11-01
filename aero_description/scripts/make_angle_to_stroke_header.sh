#!/bin/bash

# prerequisites : {my_robot}/robot.cfg
# prerequisites : {my_robot}/headers/Angle2Stroke.hh
# prerequisites : /tmp/aero_ros_order_upper < create_urdf
# prerequisites : /tmp/aero_ros_order_lower < create_urdf

# generates : aero_startup/aero_hardware_interface/Angle2Stroke.hh
# generates : aero_startup/aero_hardware_interface/Angle2Stroke.cc

dir=$1
upper_file=$2
lower_file=$3

robot_file="$(rospack find aero_description)/${dir}/robot.cfg"
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
    offset=$2
    output_file=$3
    function_name=$4
    template_file=$5
    parts_dir=$6

    # parse joint_name
    file="${parts_dir}/csv/${joint_name}.csv"

    # load csv
    code=''
    previdx=''
    code_offset=0
    while read line
    do
        idx=$(echo "$line" | cut -d ',' -f1)
        if [[ $previdx != '' ]]
        then
            if [[ $(($idx - $previdx)) -gt 1 ]]
            then
                expidx=$(($previdx + 1))
                echo "   detected idx jump in ${function_name} expecting ${expidx}, got ${idx}"
            fi
        else
            code_offset=$idx
        fi
        previdx=$idx
	e=$(echo "$line" | cut -d ',' -f4)
        val=$(echo "$offset + $e" | bc)
	interval=$(echo "$line" | cut -d ',' -f3)
        code="${code}{${val}, ${interval}},"
    done < $file
    code=${code::-1}

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

    write_declare_map=$(grep -n -m 1 "};" $output_file | cut -d ':' -f1)
    write_declare_map=$(($write_declare_map + 1))
    sed -i "${write_declare_map}i\    static const int Array${function_name}Offset = ${code_offset};" $output_file
    sed -i "${write_declare_map}i\    static const std::vector<std::pair<float, float>> ${function_name}Map = {${code}};" $output_file

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
    parts_dir=$8

    # parse joint_name_a
    file="${parts_dir}/csv/${joint_name_a}.csv"

    # load roll csv
    code1=''
    code1_offset=0
    while read line
    do
	e=$(echo "$line" | cut -d ',' -f4)
	e=$(echo "$offset_r + $e" | bc)
	interval=$(echo "$line" | cut -d ',' -f3)
        code1="${code1}{${e},${interval}}, "
    done < $file
    code1_offset=$(head -n 1 $file | cut -d ',' -f1)
    code1="${code1}"
    code1=${code1::-2}

    # parse joint_name_b
    file="${parts_dir}/csv/${joint_name_b}.csv"

    # load pitch csv
    code2=''
    code2_offset=0
    while read line
    do
	e=$(echo "$line" | cut -d ',' -f4)
	e=$(echo "$offset_p + $e" | bc)
	interval=$(echo "$line" | cut -d ',' -f3)
        code2="${code2}{${e},${interval}}, "
    done < $file
    code2_offset=$(head -n 1 $file | cut -d ',' -f1)
    code2="${code2}"
    code2=${code2::-2}

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

    write_declare_map=$(grep -n -m 1 "};" $output_file | cut -d ':' -f1)
    write_declare_map=$(($write_declare_map + 1))
    sed -i "${write_declare_map}i\    static const int Array${function_name}Offset1 = ${code1_offset};" $output_file
    sed -i "${write_declare_map}i\    static const int Array${function_name}Offset2 = ${code2_offset};" $output_file
    sed -i "${write_declare_map}i\    static const std::vector<std::pair<float, float>> ${function_name}Map1 = {${code1}};" $output_file
    sed -i "${write_declare_map}i\    static const std::vector<std::pair<float, float>> ${function_name}Map2 = {${code2}};" $output_file

    sed -i "${write_to_top_line}i\    //////////////////////////////////////////////////" $output_file
    sed -i "${write_to_top_line}i\ " $output_file
}

cp $input_file $output_file
replace_meta_in_output_file $output_file

read_csv_config() { 
    csv_file=$1
    parts_dir=$2
    total_tables=$(wc -l $csv_file | awk '{print $1}')
    for (( table_number=1; table_number<=${total_tables}; table_number++ ))
    do
        line=$(sed -n "${table_number} p" $csv_file)
        table_type=$(echo "${line}" | awk '{print $6}')
        func=$(echo "${line}" | awk '{print $1}')
        if [[ $table_type == '' ]]
        then
	    csv=$(echo "${line}" | awk '{print $3}')
	    offset=$(echo "${line}" | awk '{print $5}')
	    create_table_func_from_csv $csv $offset $output_file $func $template_file $parts_dir
        else
	    csv1=$(echo "${line}" | awk '{print $3}')
	    csv2=$(echo "${line}" | awk '{print $4}')
	    offset_p=$(echo "${line}" | awk '{print $6}')
	    offset_r=$(echo "${line}" | awk '{print $8}')
	    create_rp_table_func_from_csv $csv1 $csv2 $output_file $func $offset_p $offset_r $template_file $parts_dir
        fi
    done
}

# get required csv files

while read line
do
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
    parts_dir=$(echo $line | awk '{print $1}' | awk -F/ '{print $NF}')
    if [[ $shop_dir == "aero_shop" ]]
    then # use relative path
        parts_dir="$(rospack find aero_description)/../aero_shop/${parts_dir}"
    else
        parts_dir=$(rospack find ${shop_dir})/${parts_dir}
    fi
    # check if parts has any csv files
    if [[ $(ls $parts_dir | grep csv$) == "" ]]
    then
        continue
    fi

    csv_file="${parts_dir}/csv/Angle2Stroke.cfg"
    read_csv_config $csv_file $parts_dir
done < $robot_file

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
sed -i "/static const int Array/ d" $output_source
sed -i "/static const std::vector<std::pair<float, float>>/ d" $output_source

edit_start=$(grep -n -m 1 "//////" $output_file | cut -d ':' -f1)
tail -n +$edit_start $output_file > /tmp/aero_modify_header
sed -i "/{/,/};/ d" /tmp/aero_modify_header
sed -i "s/)/);/g" /tmp/aero_modify_header
head -n $edit_start $output_file > /tmp/aero_modify_header_head
cat /tmp/aero_modify_header_head > $output_file
cat /tmp/aero_modify_header >> $output_file
sed -i "/\/\/\/\/\/\//d" $output_file
sed -i "/#include \"aero_hardware_interface\/Angle2Stroke.hh\"/d" $output_file
