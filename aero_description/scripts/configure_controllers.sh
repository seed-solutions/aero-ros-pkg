#!/bin/bash

# prerequisites : {my_robot}/controllers.cfg

# generates : copied files listed in {my_robot}/controllers.cfg
# modifies  : aero_startup/CMakeLists.txt

function echof() {
  echo "$1" | xargs -0 -I{} sed -i "$2i\{}" $3
}
tab2=$'  '
tab6=$'      '

## FILES
robot=$1
input_file="$(rospack find aero_description)/${robot}/controllers.cfg"
cmake_file="$(rospack find aero_description)/../aero_startup/CMakeLists.txt"
# launch_file="$(rospack find aero_description)/../aero_startup/aero_bringup.launch"
launch_file="$(rospack find aero_description)/../aero_startup/generated_controllers.launch"

# create CMakeLists.txt if it does not exist
if [[ $(find $(rospack find aero_description)/../aero_startup -name "CMakeLists.txt" | grep aero_startup/CMakeLists.txt) == "" ]]
then
    cp $(rospack find aero_description)/../aero_startup/.templates/CMakeLists.template $cmake_file
fi

# create generated_controllers.launch if it does not exist
if [[ $(find $(rospack find aero_description)/../aero_startup -name "generated_controllers.launch") == "" ]]
then
    body="<launch>\n  <!"
    body="$body-- >>> add controllers -->\n  <!"
    body="$body-- <<< add controllers -->\n</launch>"
    echo -e $body > $(rospack find aero_description)/../aero_startup/generated_controllers.launch
fi

# delete controllers in launch
delete_from_launch=$(grep -n -m 1 ">>> add controllers" $launch_file | cut -d ':' -f1)
delete_from_launch=$(($delete_from_launch + 1))
delete_to_launch=$(grep -n -m 1 "<<< add controllers" $launch_file | cut -d ':' -f1)

if [[ $delete_to_launch -ne $delete_from_launch ]]
then
    delete_to_launch=$(($delete_to_launch - 1))
    sed -i "${delete_from_launch},${delete_to_launch}d" $launch_file
fi

# delete controllers in CMakeLists.txt
delete_from_line=$(grep -n -m 1 ">>> add controllers" $cmake_file | cut -d ':' -f1)
delete_from_line=$(($delete_from_line + 1))
delete_to_line=$(grep -n -m 1 "<<< add controllers" $cmake_file | cut -d ':' -f1)

if [[ $delete_to_line -ne $delete_from_line ]]
then
    delete_to_line=$(($delete_to_line - 1))
    sed -i "${delete_from_line},${delete_to_line}d" $cmake_file
fi


## GENERATE

# read from $input file
while read line
do
    header=$(echo $line | cut -d ':' -f1)
    proto=$(echo $header | awk '{print $1}')
    if [[ $proto == "#" ]] # comment out
    then
        continue
    fi
    if [[ $proto == "default" ]] # default controllers
    then
	# add aero_controller_node
	write_to_line=$(grep -n -m 1 ">>> add controllers" $cmake_file | cut -d ':' -f1)
	write_to_line=$(($write_to_line + 1))
	echof "add_executable(aero_controller_node" ${write_to_line} $cmake_file
	write_to_line=$(($write_to_line + 1))
	dir="$(rospack find aero_description)/../aero_startup/aero_hardware_interface"
	cc_files=$(find $dir -name "*.cc" | xargs -0 -I{} echo "{}" | awk -F/ '{print $NF}' | tr '\n' ' ')
	num_of_cc_files=$(find $dir -name "*.cc" | wc -l)
	for (( num=1; num<=${num_of_cc_files}; num++ ))
	do
	    file=$(echo $cc_files | awk '{print $'$num'}')
	    echof "${tab2}aero_hardware_interface/${file}" ${write_to_line} $cmake_file
	    write_to_line=$(($write_to_line + 1))
	done
	echof "${tab2})" ${write_to_line} $cmake_file
	write_to_line=$(($write_to_line + 1))
	echof "target_link_libraries(aero_controller_node \${catkin_LIBRARIES} \${Boost_LIBRARIES})\n" ${write_to_line} $cmake_file
        write_to_line=$(grep -n -m 1 ">>> add dependencies" $cmake_file | cut -d ':' -f1)
        write_to_line=$(($write_to_line + 1))
        echof "add_dependencies(aero_controller_node \${PROJECT_NAME}_gencpp)" ${write_to_line} $cmake_file

	# add aero_joint_state_publisher
	write_to_line=$(grep -n -m 1 ">>> add controllers" $cmake_file | cut -d ':' -f1)
	write_to_line=$(($write_to_line + 1))
	echof "add_executable(aero_joint_state_publisher" ${write_to_line} $cmake_file
	write_to_line=$(($write_to_line + 1))
	echof "target_link_libraries(aero_joint_state_publisher \${catkin_LIBRARIES} \${Boost_LIBRARIES})\n" ${write_to_line} $cmake_file
	echof "${tab2}aero_controller_manager/AeroJointStatePublisher.cc)" ${write_to_line} $cmake_file
	write_to_line=$(($write_to_line + 1))
	continue
    fi

    # else (not '#' or "default")
    executable_name=$(echo $header | awk '{print $2}')
    body=$(echo $line | cut -d ':' -f2)
    reference=$(echo $body | awk '{print $1}')
    executable_dir=$(echo $body | awk '{print $3}')
    robot=$(echo $reference | cut -d '/' -f1)
    source=$(echo $reference | cut -d '/' -f2)
    copy_from_file="$(rospack find aero_description)/${robot}/controllers/${source}"
    copy_to_dir="$(rospack find aero_description)/../aero_startup/${executable_dir}"
    if [[ $proto == "&" ]] # requires test (optional)
    then
	${copy_to_dir}/.test $copy_from_file
    fi
    # copy controller file
    output_file="${copy_to_dir}/${source}"
    cp $copy_from_file $output_file
    sed -i "1i\/*" $output_file
    sed -i "2i\ * This file auto-generated from script. Do not Edit!" $output_file
    sed -i "3i\ * Original : aero_description/${robot}/controllers/${source}" $output_file
    sed -i "4i\*/" $output_file

    # add executable to CMakeLists.txt
    write_to_line=$(grep -n -m 1 ">>> add controllers" $cmake_file | cut -d ':' -f1)
    write_to_line=$(($write_to_line + 1))
    echof "add_executable(aero_${executable_name}_controller_node" ${write_to_line} $cmake_file
    write_to_line=$(($write_to_line + 1))
    includes_main=$(find $copy_to_dir -name Main.cc 2>/dev/null)
    if [[ $includes_main != "" ]]
    then
	cc_files=$(find $copy_to_dir -name "*.cc" | xargs -0 -I{} echo "{}" | awk -F/ '{print $NF}' | tr '\n' ' ')
	num_of_cc_files=$(find $copy_to_dir -name "*.cc" | wc -l)
	for (( num=1; num<=${num_of_cc_files}; num++ ))
	do
	    file=$(echo $cc_files | awk '{print $'$num'}')
	    echof "${tab2}${executable_dir}/${file}" ${write_to_line} $cmake_file
	    write_to_line=$(($write_to_line + 1))
	done
	echof "${tab2})" ${write_to_line} $cmake_file
	write_to_line=$(($write_to_line + 1))
    else
	echof "${tab2}${executable_dir}/${source})" ${write_to_line} $cmake_file
	write_to_line=$(($write_to_line + 1))
    fi
    echof "target_link_libraries(aero_${executable_name}_controller_node \${catkin_LIBRARIES} \${Boost_LIBRARIES})\n" ${write_to_line} $cmake_file

    # add dependencies to CMakeLists.txt
    write_to_line=$(grep -n -m 1 ">>> add dependencies" $cmake_file | cut -d ':' -f1)
    write_to_line=$(($write_to_line + 1))
    echof "add_dependencies(aero_${executable_name}_controller_node \${PROJECT_NAME}_gencpp)" ${write_to_line} $cmake_file

    # add to launch
    echof "${tab2}<node name=\"aero_${executable_name}_controller_node\" pkg=\"aero_startup\"\n${tab2}${tab6}type=\"aero_${executable_name}_controller_node\" output=\"screen\"/>" $delete_from_launch $launch_file

done < $input_file
