#!/bin/bash

# prerequisites : {my_robot}/controllers.cfg

# generates : copied files listed in {my_robot}/controllers.cfg
# modifies  : aero_startup/CMakeLists.txt

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

tab2=$'  '

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
	echo "add_executable(aero_controller_node" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	write_to_line=$(($write_to_line + 1))
	dir="$(rospack find aero_description)/../aero_startup/aero_hardware_interface"
	cc_files=$(find $dir -name "*.cc" | xargs -0 -I{} echo "{}" | awk -F/ '{print $NF}' | tr '\n' ' ')
	num_of_cc_files=$(find $dir -name "*.cc" | wc -l)
	for (( num=1; num<=${num_of_cc_files}; num++ ))
	do
	    file=$(echo $cc_files | awk '{print $'$num'}')
	    echo "${tab2}aero_hardware_interface/${file}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	    write_to_line=$(($write_to_line + 1))
	done
	echo "${tab2})" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	write_to_line=$(($write_to_line + 1))
	echo "target_link_libraries(aero_controller_node \${catkin_LIBRARIES} \${Boost_LIBRARIES})\n" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	# add aero_joint_state_publisher
	write_to_line=$(grep -n -m 1 ">>> add controllers" $cmake_file | cut -d ':' -f1)
	write_to_line=$(($write_to_line + 1))
	echo "add_executable(aero_joint_state_publisher" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	write_to_line=$(($write_to_line + 1))
	echo "target_link_libraries(aero_joint_state_publisher \${catkin_LIBRARIES} \${Boost_LIBRARIES})\n" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	echo "${tab2}aero_controller_manager/AeroJointStatePublisher.cc)" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	write_to_line=$(($write_to_line + 1))
	continue
    fi

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
    echo "add_executable(aero_${executable_name}_controller_node" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
    write_to_line=$(($write_to_line + 1))
    includes_main=$(find $copy_to_dir -name Main.cc 2>/dev/null)
    if [[ $includes_main != "" ]]
    then
	cc_files=$(find $copy_to_dir -name "*.cc" | xargs -0 -I{} echo "{}" | awk -F/ '{print $NF}' | tr '\n' ' ')
	num_of_cc_files=$(find $copy_to_dir -name "*.cc" | wc -l)
	for (( num=1; num<=${num_of_cc_files}; num++ ))
	do
	    file=$(echo $cc_files | awk '{print $'$num'}')
	    echo "${tab2}${executable_dir}/${file}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	    write_to_line=$(($write_to_line + 1))
	done
	echo "${tab2})" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	write_to_line=$(($write_to_line + 1))
    else
	echo "${tab2}${executable_dir}/${source})" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
	write_to_line=$(($write_to_line + 1))
    fi
    echo "target_link_libraries(aero_${executable_name}_controller_node \${catkin_LIBRARIES} \${Boost_LIBRARIES})\n" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file

    # add srv files if required
    check_for_srv=$(grep "@define srv" $copy_from_file)
    if [[ "$check_for_srv" != "" ]]
    then
	# create srv file
	srv_name=$(grep "<aero_startup/" $copy_from_file | cut -d '/' -f2 | cut -d '.' -f1)
	srv_file="$(rospack find aero_startup)/srv/${srv_name}.srv"
	awk "/@define srv/,/\*\//" $copy_from_file > $srv_file
	delete_to_line=$(grep -n -m 1 "\*/" $srv_file | cut -d ':' -f1)
	sed -i "/@define/d" $srv_file
	sed -i "/\*\//d" $srv_file
	sed -i 's/^[ \t]*//' $srv_file
	# delete srv definitions from output_file
	delete_from_line=$(grep -n -m 1 "@define srv" $output_file | cut -d ':' -f1)
	delete_from_line=$(($delete_from_line - 1))
	delete_to_line=$(($delete_from_line + $delete_to_line))
	sed -i "${delete_from_line},${delete_to_line}d" $output_file
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

    # add to launch
    tab6=$'      '
    write_to_line=$delete_from_launch
    echo "${tab2}<node name=\"aero_${executable_name}_controller_node\" pkg=\"aero_startup\"" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $launch_file
    write_to_line=$(($write_to_line + 1))
    echo "${tab2}${tab6}type=\"aero_${executable_name}_controller_node\" output=\"screen\"/>" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $launch_file

done < $input_file


# add c++ dependencies :
# include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c+11" COMPILER_SUPPORTS_CXX11)
# CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# if(COMPILER_SUPPORTS_CXX11)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#   add_definitions(-DCXX11_SUPPORTED)
# elseif(COMPILER_SUPPORTS_CXX0X)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#   add_definitions(-DCXX11_SUPPORTED)
# else()
#   message(FATAL "c++11 required but not supported")
# endif()
# end dep

awk "/# add c\+\+ dependencies :/,/# end dep/" $(rospack find aero_description)/scripts/configure_controllers.sh > /tmp/cxx11dep
sed -i "/# add c\+\+/d" /tmp/cxx11dep
sed -i "/# end dep/d" /tmp/cxx11dep
sed -i "s/# //g" /tmp/cxx11dep
write_to_line=$(grep -n -m 1 ">>> add dependencies" $cmake_file | cut -d ':' -f1)
write_to_line=$(($write_to_line + 1))
IFS=''
while read line
do
    echo "${line}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
    write_to_line=$(($write_to_line + 1))
done < /tmp/cxx11dep
