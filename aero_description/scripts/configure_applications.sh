#!/bin/bash

# prerequisites : {my_robot}/applications.cfg

# modifies  : aero_startup/CMakeLists.txt

robot=$1
input_file="$(rospack find aero_description)/${robot}/applications.cfg"
cmake_file="$(rospack find aero_description)/../aero_startup/CMakeLists.txt"
launch_file="$(rospack find aero_description)/../aero_startup/applications.launch"

# delete applications in launch

delete_from_launch=$(grep -n -m 1 ">>> add applications" $launch_file | cut -d ':' -f1)
delete_from_launch=$(($delete_from_launch + 1))
delete_to_launch=$(grep -n -m 1 "<<< add applications" $launch_file | cut -d ':' -f1)

if [[ $delete_to_launch -ne $delete_from_launch ]]
then
    delete_to_launch=$(($delete_to_launch - 1))
    sed -i "${delete_from_launch},${delete_to_launch}d" $launch_file
fi

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

while read line
do
    header=$(echo $line | cut -d: -f1)
    proto=$(echo $header | awk '{print $1}')
    if [[ $proto == "#" ]] # comment out
    then
	continue
    fi
    executable_name=$(echo $header | awk '{print $2}')
    body=$(echo $line | cut -d: -f2)
    executable_dir=$(echo $body | awk '{print $1}')
    source=$(echo $executable_name)
    executable_path="$(rospack find aero_description)/../aero_startup/${executable_dir}"
    # rename source depending on camel letters
    find_source=$(find ${executable_path} -name "${source}*")
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
    fi
    source=$(echo "${find_source}" | awk -F/ '{print $NF}')

    # add to launch
    tab6=$'      '
    write_to_line=$delete_from_launch
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
	dependencies=$(echo $line | cut -d: -f3)
	num_of_dependencies=$(echo $dependencies | awk '{print NF}')
	dependency_list="$(rospack find aero_description)/../aero_startup/.dependencies"
	for (( num=1; num<=${num_of_dependencies}; num++ ))
	do
	    target=$(echo $dependencies | awk '{print $'$num'}')
	    libs="${libs}$(grep "${target}" $dependency_list | awk '{print $2}')"
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
	echo "${tab}add_executable(${executable_name}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	write_to_line=$(($write_to_line + 1))
	includes_main=$(find $executable_dir -name Main.cc 2>/dev/null)
	if [[ $includes_main != "" ]]
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
	else
	    echo "${tab}${tab2}${executable_dir}/${source})" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	    write_to_line=$(($write_to_line + 1))
	fi
	echo "${tab}target_link_libraries(${executable_name} \${catkin_LIBRARIES} \${Boost_LIBRARIES}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	write_to_line=$(($write_to_line + 1))
	echo "${tab}${tab2}${libs})" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	if [[ $ifs != "" ]]
	then
	    write_to_line=$(($write_to_line + 1))
	    echo "endif()" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	fi

	# add srv files if required
	check_for_srv=$(grep "@define srv" "${executable_path}/${source}")
	if [[ "$check_for_srv" != "" ]]
	then
	    # create srv file
	    mkdir $(rospack find aero_startup)/srv
	    srv_name=$(grep "<aero_startup/" $source | cut -d '/' -f2 | cut -d '.' -f1)
	    srv_file="$(rospack find aero_startup)/srv/${srv_name}.srv"
	    awk "/@define srv/,/\*\//" $source > $srv_file
	    delete_to_line=$(grep -n -m 1 "\*/" $srv_file | cut -d ':' -f1)
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
	fi
    fi
done < $input_file
