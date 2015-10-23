#!/bin/bash

input_file="$(rospack find aero_ros_bridge)/templates/Stroke2Angle.hh"
output_file="$(rospack find aero_ros_bridge)/../aero_world/src/Stroke2Angle.hh"
cp $input_file $output_file

total_joints=$(grep -o 'meta =' $output_file | wc -l)
for (( joint_number=0; joint_number<=${total_joints}; joint_number++ ))
do
    sed -i "0,/meta =/s//_angles[${joint_number}] =/" $output_file
done

hrpsys_names_file="$(rospack find aero_ros_bridge)/config/hrpsys_joint_id_map.txt"
number=0
while read line
do
    name=$(echo "$line" | cut -d ' ' -f1)
    sed -i "s/${name}/_strokes[${number}]/g" $output_file
    number=$(($number + 1))
done < $hrpsys_names_file


create_table_func_from_csv() { # joint_name offset output_file function_name
    # load csv
    csv=()
    csv_interval=()
    file="$(rospack find aero_ros_bridge)/models/${1}.csv"
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

    awk "/float TableTemplate/,/};/" $3 > /tmp/mjointsanglehh
    sed -i "s/TableTemplate/${1}/g" /tmp/mjointsanglehh
    write_to_line=$(grep -n -m 1 "switch (roundedStroke)" /tmp/mjointsanglehh | cut -d ':' -f1)
    write_to_line=$(($write_to_line + 2))

    echo -e "${code}" > /tmp/mjointsanglehhwrite
    sed -i '1!G;h;$!d' /tmp/mjointsanglehhwrite

    IFS=''
    while read line
    do
    	echo "${line}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" /tmp/mjointsanglehh
    done < /tmp/mjointsanglehhwrite

    write_to_line=$(grep -n -m 1 "#endif" $3 | cut -d ':' -f1)
    write_to_line=$(($write_to_line - 4))
    sed -i '1!G;h;$!d' /tmp/mjointsanglehh

    IFS=''
    while read line
    do
    	echo "${line}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $3
    done < /tmp/mjointsanglehh

    sed -i "${write_to_line}i\    //////////////////////////////////////////////////" $3
    sed -i "${write_to_line}i\ " $3
    sed -i "s/${1}/${4}/g" $3
}

create_table_func_from_csv shoulder-p 0 $output_file ShoulderPitchInvTable
create_table_func_from_csv shoulder-r 0 $output_file ShoulderRollInvTable
create_table_func_from_csv elbow-p 20.183 $output_file ElbowPitchInvTable
create_table_func_from_csv wrist-p 0 $output_file WristPitchInvTable
create_table_func_from_csv wrist-r 0 $output_file WristRollInvTable
create_table_func_from_csv waist-p 0 $output_file WaistPitchInvTable
create_table_func_from_csv waist-r 0 $output_file WaistRollInvTable
create_table_func_from_csv neck-p 0 $output_file NeckPitchInvTable
create_table_func_from_csv neck-r 0 $output_file NeckRollInvTable
create_table_func_from_csv crotch-p 0 $output_file CrotchPitchInvTable
create_table_func_from_csv knee-p 0 $output_file KneePitchInvTable

sed -i "/float TableTemplate/,/};/ d" $output_file

# from here replace position of Stroke2Angle
awk "/void Stroke2Angle/,/};/" $output_file > /tmp/mjointsanglehh
sed -i "/void Stroke2Angle/,/};/ d" $output_file
write_to_line=$(grep -n -m 1 "#endif" $output_file | cut -d ':' -f1)
write_to_line=$(($write_to_line - 4))
sed -i '1!G;h;$!d' /tmp/mjointsanglehh

IFS=''
while read line
do
    echo "${line}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $output_file
done < /tmp/mjointsanglehh

sed -i "${write_to_line}i\    //////////////////////////////////////////////////" $output_file
sed -i "${write_to_line}i\ " $output_file
