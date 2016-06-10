#!/bin/bash

srv_dir="$(rospack find aero_description)/../aero_startup/srv"
cmake_file="$(rospack find aero_description)/../aero_startup/CMakeLists.txt"

# create CMakeLists.txt if not exist
# clean or overwrite if exists

if [[ $(find $(rospack find aero_description)/../aero_startup -name "CMakeLists.txt" | grep aero_startup/CMakeLists.txt) == "" ]]
then
    cp $(rospack find aero_description)/../aero_startup/.templates/CMakeLists.template $cmake_file
else
    echo "clean CMakeLists.txt? y or n"
    read answer
    case $answer in
	y)
	    cp $(rospack find aero_description)/../aero_startup/.templates/CMakeLists.template $cmake_file
	    echo "cleaned"
	    ;;
	*)
	    echo "overwriting without cleaning"
	    ;;
    esac
fi

# add srvs to CMakeLists.txt

ls_srv=$(ls $srv_dir | tr '\n' ' ')
num_of_srvs=$(ls $srv_dir | wc -l)
tab2=$'  '

for (( num=1; num<=$num_of_srvs; num++))
do
    target=$(echo $ls_srv | awk '{print $'$num'}')
    check_if_exists=$(grep "${target}" $cmake_file)
    if [[ "$check_if_exists" == "" ]]
    then
        sed -i "s/set(GENERATE_SRV)/set(GENERATE_SRV 1)/g" $cmake_file
        write_to_line=$(grep -n -m 1 "auto-add services" $cmake_file | cut -d ':' -f1)
        write_to_line=$(($write_to_line + 3))
        echo "${tab2}${tab2}${target}" | xargs -0 -I{} sed -i "${write_to_line}i\{}" $cmake_file
    fi
done
