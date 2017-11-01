#!/bin/bash

# prerequisites : {my_robot}/robot.cfg
# prerequisites : {my_robot}/headers/Stroke2Angle.hh
# prerequisites : /tmp/aero_CAN_order < make_controller

# generates : aero_startup/aero_hardware_interface/Stroke2Angle.hh
# generates : aero_startup/aero_hardware_interface/Stroke2Angle.cc

dir=$1

robot_file="$(rospack find aero_description)/${dir}/robot.cfg"
input_file="$(rospack find aero_description)/${dir}/headers/Stroke2Angle.hh"
output_file="$(rospack find aero_description)/../aero_startup/aero_hardware_interface/Stroke2Angle.hh"
output_source="$(rospack find aero_description)/../aero_startup/aero_hardware_interface/Stroke2Angle.cc"
template_file="$(rospack find aero_description)/../aero_startup/.templates/aero_hardware_interface/Stroke2Angle.hh"

replace_meta_in_output_file() {
    output_file=$1

    total_joints=$(grep -o 'meta =' $output_file | wc -l)
    for (( joint_number=0; joint_number<=${total_joints}; joint_number++ ))
    do
	sed -i "0,/meta =/s//_angles[${joint_number}] =/" $output_file
    done

    number=0
    while read line
    do
	name=$(echo "$line" | cut -d ' ' -f1)
	sed -i "s/${name}/_strokes[${number}]/g" $output_file
	number=$(($number + 1))
    done < /tmp/aero_CAN_order
}

create_table_func_from_csv() {
    joint_name=$1
    # offset=$2
    output_file=$3
    function_name=$4
    template_file=$5
    parts_dir=$6

    # parse joint_name
    file="${parts_dir}/csv/${joint_name}.csv"

    # load csv
    csv=()
    csv_interval=()
    j=0
    while read line
    do
        Angle[j]=$(echo "$line" | cut -d ',' -f1)
	csv[j]=$(echo "$line" | cut -d ',' -f4)
	csv_interval[j]=$(echo "$line" | cut -d ',' -f3)
	j=$(($j + 1))
    done < $file

    # preparation for stroke-angle-table : organize csv according to integral
    table=()
    ntable=()
    ntable[0]=""
    idx=0
    bef_head=""
    for e in "${csv[@]}"
    do
	val=$(echo "$2 + $e" | bc)
	head=$(echo "${val}" | sed s/\.[0-9,]*$//g)

	# getting rid of possible bugs
	if [[ $head == "-" ]]
	then
	    head=0
	elif [[ $head == "" ]]
	then
	    head=0
	fi

	if [[ $head -lt 0 ]] # when stroke is negative
	then
	    head=$(echo "-1 * $head" | bc)
	    # if +1 degree results to more than +1 stroke
	    if [[ ${bef_head} != ""  ]]
	    then
		if [[ $(echo "$head - $bef_head" | bc ) -gt 1 ]]
		then
		    ntable[$(($head - 1))]=""
		fi
	    fi
	    bef_head=$head
	    ntable[$head]="${ntable[$head]}{${Angle[${idx}]},${val},${csv_interval[${idx}]}},"
	else # when stroke is positive
	    # if +1 degree results to more than +1 stroke
	    if [[ ${bef_head} != ""  ]]
	    then
		if [[ $(echo "$head - $bef_head" | bc ) -gt 1 ]]
		then
		    table[$(($head - 1))]=""
		fi
	    fi
	    bef_head=$head

	    table[$head]="${table[$head]}{${Angle[${idx}]},${val},${csv_interval[${idx}]}},"
	fi
	idx=$(($idx + 1))
    done

    code=''
    array_offset=0

    # negative stroke value case
    if [[ ${#ntable[@]} -gt 0 ]]
    then
        idx=$((${#ntable[@]} - 1))
        e=${ntable[$idx]}
        if [[ $e != "" ]]
        then
            code="${code}{{${e::-1}},{}}, "
            array_offset="-$idx"
        elif [[ ${#ntable[@]} -gt 1 ]]
        then
            idx=$(($idx - 1))
            array_offset="-$idx"
        fi
    fi
    for (( idx=${#ntable[@]}-2 ; idx>=0 ; idx-- ))
    do
        e=${ntable[$idx]}
	if [[ $e != "" ]]
	then
            code="${code}{{${e::-1}},"
	    j=1
	    if [[ ${ntable[$(($idx + $j))]} == "" ]]
	    then
		j=2
	    fi
	    appendix=$(echo -e "${ntable[$(($idx + $j))]}")
            if [[ "$appendix" = "" ]]
            then
                code="${code}{}}, "
            else
		code="${code}{${appendix::-1}}}, "
            fi
        elif [[ $idx != 0 ]]
        then
            echo "   detected empty table in -${idx} of ${function_name}"
            code="${code}{{{0,0.0f,0.0f}}, {}}, "
	fi
    done

    # positive stroke value case
    idx=0
    for e in "${table[@]}"
    do
	if [[ $e != "" ]]
	then
            code="${code}{{${e::-1}},"
	    if [[ $idx -lt $((${#table[@]} - 1)) ]]
	    then
		appendix=$(echo -e "${table[$(($idx + 1))]}")
                if [[ "$appendix" = "" ]]
                then
                    code="${code}{}}, "
                else
		    code="${code}{${appendix::-1}}}, "
                fi
            else
                code="${code}{}}, "
	    fi
        else
            echo "   detected empty table in ${idx} of ${function_name}"
            code="${code}{{{0,0.0f,0.0f}}, {}}, "
	fi
	idx=$(($idx + 1))
    done
    code="${code::-2}"

    awk "/float TableTemplate/,/};/" $template_file > /tmp/mjointsanglehh
    sed -i "s/TableTemplate/${function_name}/g" /tmp/mjointsanglehh

    write_to_line=$(grep -n -m 1 "void Stroke2Angle" $output_file | cut -d ':' -f1)
    write_to_line=$(($write_to_line - 1))
    write_to_top_line=$write_to_line

    IFS=''
    while read line
    do
    	echo "${line}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $output_file
	write_to_line=$(($write_to_line + 1))
    done < /tmp/mjointsanglehh

    write_declare_map=$(grep -n -m 1 "};" $output_file | cut -d ':' -f1)
    write_declare_map=$(($write_declare_map + 1))
    sed -i "${write_declare_map}i\    static const int Array${function_name}Offset = ${array_offset};" $output_file
    sed -i "${write_declare_map}i\    static const std::vector<std::pair<std::vector<S2AData>, std::vector<S2AData>>> ${function_name}Candidates = {${code}};" $output_file

    sed -i "${write_to_top_line}i\    //////////////////////////////////////////////////" $output_file
    sed -i "${write_to_top_line}i\ " $output_file

    echo -e "created ${function_name}"
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
        csv=$(echo "${line}" | awk '{print $3}')
        func=$(echo "${line}" | awk '{print $1}')
        offset=$(echo "${line}" | awk '{print $5}')
        create_table_func_from_csv $csv $offset $output_file $func $template_file $parts_dir
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
    parts_dir=$(echo $line | awk '{print $1}' | cut -d/ -f2)
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

    csv_file="${parts_dir}/csv/Stroke2Angle.cfg"
    read_csv_config $csv_file $parts_dir
done < $robot_file

# write warnings

sed -i "1i\/*" $output_file
sed -i "2i\ * This file auto-generated from script. Do not Edit!" $output_file
sed -i "3i\ * Original : aero_startup/.templates/aero_hardware_interface/Stroke2Angle.hh" $output_file
sed -i "4i\ * Original : aero_description/{my_robot}/headers/Stroke2Angle.hh" $output_file
sed -i "5i\*/" $output_file

# divide header and source

cp $output_file $output_source
sed -i "/struct/,/};/ d" $output_source
sed -i "/#ifndef/d" $output_source
sed -i "/#define/d" $output_source
sed -i "/#endif/d" $output_source
sed -i "s/};/}/g" $output_source
sed -i "/static const int Array/ d" $output_source
sed -i "/static const std::vector<std::pair<std::vector<S2AData>, std::vector<S2AData>>>/ d" $output_source

edit_start=$(grep -n -m 1 "//////" $output_file | cut -d ':' -f1)
tail -n +$edit_start $output_file > /tmp/aero_modify_header
sed -i "/{/,/};/ d" /tmp/aero_modify_header
sed -i "s/)/);/g" /tmp/aero_modify_header
head -n $edit_start $output_file > /tmp/aero_modify_header_head
cat /tmp/aero_modify_header_head > $output_file
cat /tmp/aero_modify_header >> $output_file
sed -i "/\/\/\/\/\/\//d" $output_file
sed -i "/#include \"aero_hardware_interface\/Stroke2Angle.hh\"/d" $output_file
