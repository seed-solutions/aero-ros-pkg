#!/bin/bash

# prerequisites : {my_robot}/applications.cfg

# modifies  : aero_startup/CMakeLists.txt

app=$1
robot=$2

if [[ $robot == "" ]]
then
    input_file=$(find $(rospack find aero_description)/ -name "${app}.cfg")
    if [[ $(find $(rospack find aero_description)/ -name "${app}.cfg" | wc -l) -ne 1 ]]
    then
	echo "aborting, robot not specified or file not found"
	exit
    fi
else
    input_file="$(rospack find aero_description)/${robot}/${app}.cfg"
fi

cmake_file="$(rospack find aero_description)/../aero_startup/CMakeLists.txt"
launch_file="$(rospack find aero_description)/../aero_startup/${app}.launch"

# create launch

echo -e "<launch>\n</launch>" > $launch_file

# delete applications in CMakeLists.txt

delete_from_line=$(grep -n -m 1 ">>> add applications" $cmake_file | cut -d ':' -f1)
delete_from_line=$(($delete_from_line + 1))
delete_to_line=$(grep -n -m 1 "<<< add applications" $cmake_file | cut -d ':' -f1)

if [[ $delete_to_line -ne $delete_from_line ]]
then
    delete_to_line=$(($delete_to_line - 1))
    sed -i "${delete_from_line},${delete_to_line}d" $cmake_file
fi

tab2=$'  '

create_srv_file() {
    lookup_file=$1

    check_for_srv=$(grep "@define srv" $1)
    if [[ "$check_for_srv" != "" ]]
    then
	# check num of srvs
	num_of_srvs=$(grep "@define srv" $1 | wc -l)
	# create srv file
	for (( srv=1; srv<=${num_of_srvs}; srv++ ))
	do
	    srv_name=$(awk "/<aero_startup\//{i++}i==${srv}{print; exit}" $1 | cut -d '/' -f2 | cut -d '.' -f1)
	    srv_file="$(rospack find aero_startup)/srv/${srv_name}.srv"
	    if [[ "$num_of_srvs" -gt "1" ]]
	    then
		awk "/@define srv ${srv}/,/\*\//" $1 > $srv_file
	    else
		awk "/@define srv/,/\*\//" $1 > $srv_file
	    fi
	    sed -i "/@define/d" $srv_file
	    sed -i "/\*\//d" $srv_file
	    sed -i 's/^[ \t]*//' $srv_file
	    # add srv generation to CMakeLists.txt
	    check_if_exists=$(grep "${srv_name}.srv" $cmake_file)
	    if [[ "$check_if_exists" == "" ]]
	    then
		sed -i "s/set(GENERATE_SRV)/set(GENERATE_SRV 1)/g" $cmake_file
		write_to_line=$(grep -n -m 1 "auto-add services" $cmake_file | cut -d ':' -f1)
		write_to_line=$(($write_to_line + 3))
		echo "${tab2}${tab2}${srv_name}.srv" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	    fi
	done
    fi
}

executable_name=""
node_args=0
while read line
do
    header=$(echo $line | cut -d: -f1)
    proto=$(echo $header | awk '{print $1}')
    if [[ $proto == "#" ]] # comment out
    then
	continue
    elif [[ $proto == "remap" ]]
    then
	from=$(echo $line | awk '{print $2}')
	to=$(echo $line | awk '{print $4}')
	sed -i "s@type=\"${executable_name}\" output=\"screen\"/>@type=\"${executable_name}\" output=\"screen\">@" $launch_file
	write_to_line=$(grep -n -m 1 "type=\"${executable_name}\" output=\"screen\">" $launch_file | cut -d: -f1)
	write_to_line=$(($write_to_line + 1))
	echo "${tab2}${tab2}<remap from=\"${from}\" to=\"${to}\"/>" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $launch_file
	node_args=$(($node_args + 1))
	node_end_line=$(($write_to_line + $node_args))
	has_node_end=$(sed -n "${node_end_line}p" $launch_file | grep "</node>")
	if [[ $has_node_end == "" ]]
	then
	    write_to_line=$(($write_to_line + 1))
	    echo "${tab2}</node>" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $launch_file
	fi
	continue
    fi
    node_args=0
    executable_name=$(echo $header | awk '{print $2}')
    body=$(echo $line | cut -d: -f2)
    executable_dir=$(echo $body | awk '{print $1}')
    source=$(echo $executable_name)
    executable_path="$(rospack find aero_description)/../aero_startup/${executable_dir}"
    # rename source depending on camel letters
    find_source=$(find ${executable_path} -name "${source}*")
    has_same_name_class_object=$(find ${executable_path} -name "${source}*" | grep "class")
    if [[ $find_source == "" ]]
    then
	num_of_words=$(echo $source | awk -F_ '{print NF}')
	file_name=''
	for (( num=1; num<=$num_of_words; num++ ))
	do
	    word=$(echo $source | awk -F_ '{print $'$num'}')
	    file_name="${file_name}${word^}"
	done
	find_source=$(find ${executable_path} -name "${file_name}*")
	has_same_name_class_object=$(find ${executable_path} -name "${file_name}*" | grep "/class/")
    fi
    # if class object with same name as executable exists
    if [[ $has_same_name_class_object != "" ]]
    then
	find_source=$(find ${executable_path} -name "${file_name}*" | grep -v "/class/") 
    fi
    source=$(echo "${find_source}" | awk -F/ '{print $NF}')

    # add to launch
    tab6=$'      '
    write_to_line=2
    if [[ $executable_name == *".launch" ]]
    then
	echo "${tab2}<include file=\"\$(find aero_startup)/${executable_dir}/launch/${executable_name}\"/>" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $launch_file
    else
	echo "${tab2}<node name=\"${executable_name}\" pkg=\"aero_startup\"" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $launch_file
	write_to_line=$(($write_to_line + 1))
	echo "${tab2}${tab6}type=\"${executable_name}\" output=\"screen\"/>" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $launch_file
    fi

    if [[ $proto == "&" ]] # requires special installation script
    then
	install=$(echo $line | cut -d: -f3)
	${executable_path}/.install $install

    elif [[ $proto == "+" ]]
    then
	if [[ $executable_name == *".launch" ]]
	then
	    continue
	fi

	# add executable to CMakeLists.txt
	libs=''
	ifs=''

	# dependecy check
	dependencies=$(echo $line | cut -d: -f3)
	num_of_dependencies=$(echo $dependencies | awk '{print NF}')
	dependency_list="$(rospack find aero_description)/../aero_startup/.dependencies"
	for (( num=1; num<=${num_of_dependencies}; num++ ))
	do
	    target=$(echo $dependencies | awk '{print $'$num'}')
	    libs="${libs} $(grep "${target}" $dependency_list | awk '{print $2}')"
	    if [[ $ifs == "" ]]
	    then
		ifs="FOUND_${target}"
	    else
		ifs="${ifs} AND FOUND_${target}"
	    fi
	done
	write_to_line=$(grep -n -m 1 ">>> add applications" $cmake_file | cut -d ':' -f1)
	write_to_line=$(($write_to_line + 1))
	tab=''
	if [[ $ifs != "" ]]
	then
	    tab=$'  '
	    echo "if(${ifs})" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	    write_to_line=$(($write_to_line + 1))
	fi

	# add_executable
	echo "${tab}add_executable(${executable_name}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	write_to_line=$(($write_to_line + 1))
	includes_main=$(find $executable_dir -name Main.cc 2>/dev/null)
	include_class_objects=$(grep "/class/" $find_source)
	class_files=""
	num_of_class_files=""
	if [[ $includes_main != "" ]] # if executable is Main.cc
	then
	    cc_files=$(find $copy_to_dir -name "*.cc" | xargs -0 -I{} echo "{}" | awk -F/ '{print $NF}')
	    num_of_cc_files=$(find $copy_to_dir -name "*.cc" | wc -l)
	    for (( num=1; num<=${num_of_cc_files}; num++ ))
	    do
		file=$(echo $cc_files | awk '{print $'$num'}')
		echo "${tab}${tab2}${executable_dir}/${file}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
		write_to_line=$(($write_to_line + 1))
	    done
	    echo "${tab}${tab2})" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	    write_to_line=$(($write_to_line + 1))
	elif [[ $include_class_objects != "" ]] # if relies on class objects
	then
	    class_files=$(grep "#include \"aero_" $find_source | grep "/class/" | awk '{print $2}' | cut -d\" -f2)
	    num_of_class_files=$(grep "#include \"aero_" $find_source | grep "class" | wc -l)
	    for (( num=1; num<=${num_of_class_files}; num++ ))
	    do
		file=$(echo $class_files | awk '{print $'$num'}' | sed "s/.hh/.cc/g")
		echo "${tab}${tab2}${file}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
		write_to_line=$(($write_to_line + 1))
	    done
	    echo "${tab}${tab2}${executable_dir}/${source})" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	    write_to_line=$(($write_to_line + 1))
	else
	    echo "${tab}${tab2}${executable_dir}/${source})" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	    write_to_line=$(($write_to_line + 1))
	fi

	# target link
	echo "${tab}target_link_libraries(${executable_name} \${catkin_LIBRARIES} \${Boost_LIBRARIES}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	write_to_line=$(($write_to_line + 1))
	echo "${tab}${tab2}${libs})" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	if [[ $ifs != "" ]]
	then
	    write_to_line=$(($write_to_line + 1))
	    echo "endif()\n" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	else
	    write_to_line=$(($write_to_line + 1))
	    echo "" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	fi

	# add srv files if required
	create_srv_file "${executable_path}/${source}"
	if [[ $class_files != "" ]]
	then
	    for (( num=1; num<=${num_of_class_files}; num++ ))
	    do
		file=$(echo $class_files | awk '{print $'$num'}')
		create_srv_file "$(rospack find aero_description)/../aero_startup/${file}"
	    done
	fi
    fi
done < $input_file
