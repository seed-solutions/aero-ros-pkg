#!/bin/bash

# prerequisites : {my_robot}/headers/Stroke2Angle.hh
# prerequisites : /tmp/aero_CAN_order < make_controller

# generates : aero_startup/aero_common/Stroke2Angle.hh
# generates : aero_startup/aero_common/Stroke2Angle.cc

dir=$1

input_file="$(rospack find aero_description)/${dir}/headers/Stroke2Angle.hh"
output_file="$(rospack find aero_description)/../aero_startup/aero_common/Stroke2Angle.hh"
output_source="$(rospack find aero_description)/../aero_startup/aero_common/Stroke2Angle.cc"
template_file="$(rospack find aero_description)/../aero_startup/.templates/aero_common/Stroke2Angle.hh"

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

    # load csv
    csv=()
    csv_interval=()
    file="$(rospack find aero_description)/models/${joint_name}.csv"
    j=0
    while read line
    do
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
	    ntable[$head]="${ntable[$head]}{${idx}, ${val}, ${csv_interval[${idx}]}}, "
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

	    table[$head]="${table[$head]}{${idx}, ${val}, ${csv_interval[${idx}]}}, "
	fi
	idx=$(($idx + 1))
    done

    tab6=$'      '
    tab8=$'        '
    code=''

    # negative stroke value case
    idx=0
    for e in "${ntable[@]}"
    do
	if [[ $e != "" ]]
	then
	    code="${code}${tab6}case -${idx}:\n"
	    candidates=$(echo -e "${e}" | sed -e "s/..$//")
	    code="${code}${tab8}candidates = std::vector<S2AData>({${candidates}});\n"
	    if [[ $idx -lt $((${#ntable[@]} - 1)) ]]
	    then
		j=1
		if [[ ${ntable[$(($idx + $j))]} == "" ]]
		then
		    j=2
		fi
		appendix=$(echo -e "${ntable[$(($idx + $j))]}" | sed -e "s/..$//")
		code="${code}${tab8}appendix = std::vector<S2AData>({${appendix}});\n"
	    fi
	    code="${code}${tab8}break;\n"
	fi
	idx=$(($idx + 1))
    done

    # positive stroke value case
    idx=0
    for e in "${table[@]}"
    do
	if [[ $e != "" ]]
	then
	    code="${code}${tab6}case ${idx}:\n"
	    candidates=$(echo -e "${e}" | sed -e "s/..$//")
	    code="${code}${tab8}candidates = std::vector<S2AData>({${candidates}});\n"
	    if [[ $idx -lt $((${#table[@]} - 1)) ]]
	    then
		appendix=$(echo -e "${table[$(($idx + 1))]}" | sed -e "s/..$//")
		code="${code}${tab8}appendix = std::vector<S2AData>({${appendix}});\n"
	    fi
	    code="${code}${tab8}break;\n"
	fi
	idx=$(($idx + 1))
    done

    echo -e "${code}" > /tmp/mjointsanglehhwrite

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

    write_to_line=$(grep -n -m 1 "switch (roundedStroke)" /tmp/mjointsanglehh | cut -d ':' -f1)
    write_to_line=$(($write_to_line + 1))
    write_to_line=$(($write_to_top_line + $write_to_line))

    IFS=''
    while read line
    do
    	echo "${line}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $output_file
	write_to_line=$(($write_to_line + 1))
    done < /tmp/mjointsanglehhwrite

    sed -i "${write_to_top_line}i\    //////////////////////////////////////////////////" $output_file
    sed -i "${write_to_top_line}i\ " $output_file
}

cp $input_file $output_file
replace_meta_in_output_file $output_file

total_tables=$(grep -o '@define' $output_file | wc -l)
for (( table_number=1; table_number<=${total_tables}; table_number++ ))
do
    line=$(awk "/@define/{i++}i==${table_number}{print; exit}" $output_file)
    csv=$(echo "${line}" | awk '{print $4}')
    func=$(echo "${line}" | awk '{print $2}')
    offset=$(echo "${line}" | awk '{print $6}')
    create_table_func_from_csv $csv $offset $output_file $func $template_file
done

sed -i "1,$((${total_tables} + 2))d" $output_file

# write warnings

sed -i "1i\/*" $output_file
sed -i "2i\ * This file auto-generated from script. Do not Edit!" $output_file
sed -i "3i\ * Original : aero_startup/.templates/aero_common/Stroke2Angle.hh" $output_file
sed -i "4i\ * Original : aero_description/{my_robot}/headers/Stroke2Angle.hh" $output_file
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
sed -i "/#include \"aero_common\/Stroke2Angle.hh\"/d" $output_file
