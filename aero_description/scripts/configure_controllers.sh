#!/bin/bash

# version v2.01

# prerequisites : {my_robot}/robot.cfg

# generates : controllers in {my_shop}/{my_parts}/controller
# modifies  : aero_startup/CMakeLists.txt

## @brief insert string into file
## @param $1 string to insert
## @param $2 line number to insert
## @param $3 file to insert
function echof() {
  echo "$1" | xargs -0 -I{} sed -i "$2i\{}" $3
}

tab2=$'  '
tab6=$'      '

## FILES
robot=$1
input_file="$(rospack find aero_description)/${robot}/robot.cfg"
cmake_file="$(rospack find aero_description)/../aero_startup/CMakeLists.txt"
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

## @brief delete lines from >>> to <<<
## @param $1 section
## @param $2 target file
function delete_section_from() {
    delete_from=$(grep -n -m 1 ">>> add $1" $2 | cut -d ':' -f1)
    delete_from=$(($delete_from + 1))
    delete_to=$(grep -n -m 1 "<<< add $1" $2 | cut -d ':' -f1)

    if [[ $delete_to -ne $delete_from ]]
    then
	delete_to=$(($delete_to - 1))
	sed -i "${delete_from},${delete_to}d" $2
    fi
}
## @brief delete controllers
## @param $1 target file
function delete_controllers_from() {
    delete_section_from "controllers" $1
}
## @brief delete dependencies
## @param $1 target file
function delete_dependencies_from() {
    delete_section_from "dependencies" $1
}

# delete controllers in launch
delete_controllers_from $launch_file

# delete controllers and dependencies in CMakeLists.txt
delete_controllers_from $cmake_file
delete_dependencies_from $cmake_file


## GENERATE

# default controllers (always created as of v2.01)

# add aero_controller_node
write_to_line=$(grep -n -m 1 ">>> add controllers" $cmake_file | cut -d ':' -f1)
write_to_line=$(($write_to_line + 1))
echof "add_executable(aero_controller_node" ${write_to_line} $cmake_file
write_to_line=$(($write_to_line + 1))
# enumerate *.cc files in aero_startup into CMakeLists.txt
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

# read from $input file
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

    # else (not '#' or body name)
    executable_dir=$(echo $line | awk '{print $3}')
    if [[ $executable_dir == "" ]]
    then
        executable_dir="aero_controller_manager"
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
    # check if parts has any controller
    if [[ $(ls $parts_dir | grep controller$) == "" ]]
    then
        continue
    fi

    controller_dir="${parts_dir}/controller"
    executables=$(ls $controller_dir)
    num_of_executables=$(ls $controller_dir | wc -l)
    for (( i=1 ; i<=${num_of_executables}; i++ ))
    do
        source=$(echo $executables | awk '{print $i}')
        executable_name=$(echo $source | cut -d. -f1 | awk '{print tolower($0)}')
        copy_from_file="${controller_dir}/${source}"
        copy_to_dir="$(rospack find aero_description)/../aero_startup/${executable_dir}"
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
        echof "add_executable(${executable_name}_node" ${write_to_line} $cmake_file
        write_to_line=$(($write_to_line + 1))
        includes_main=$(find $copy_to_dir -name Main.cc 2>/dev/null)
        if [[ $includes_main != "" ]]
        then
	    # enumerate *.cc files in aero_startup into CMakeLists.txt
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
        echof "target_link_libraries(${executable_name}_node \${catkin_LIBRARIES} \${Boost_LIBRARIES})\n" ${write_to_line} $cmake_file

        # add dependencies to CMakeLists.txt
        write_to_line=$(grep -n -m 1 ">>> add dependencies" $cmake_file | cut -d ':' -f1)
        write_to_line=$(($write_to_line + 1))
        echof "add_dependencies(${executable_name}_node \${PROJECT_NAME}_gencpp)" ${write_to_line} $cmake_file

        # add to launch
        write_to_line=$(grep -n -m 1 ">>> add controllers" $launch_file | cut -d ':' -f1)
        write_to_line=$(($write_to_line + 1))
        echof "${tab2}<node name=\"${executable_name}_node\" pkg=\"aero_startup\"\n${tab2}${tab6}type=\"${executable_name}_node\" output=\"screen\"/>" ${write_to_line} $launch_file
    done

done < $input_file
