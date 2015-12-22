#!/bin/bash

srv_dir="$(rospack find aero_description)/../aero_startup/srv"
cmake_file="$(rospack find aero_description)/../aero_startup/CMakeLists.txt"

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
