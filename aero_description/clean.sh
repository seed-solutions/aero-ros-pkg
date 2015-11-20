#!/bin/bash

cmake_file="$(rospack find aero_description)/../aero_startup/CMakeLists.txt"

# delete controllers from CMakeLists.txt

delete_from_line=$(grep -n -m 1 ">>> add controllers" $cmake_file | cut -d ':' -f1)
delete_from_line=$(($delete_from_line + 1))
delete_to_line=$(grep -n -m 1 "<<< add controllers" $cmake_file | cut -d ':' -f1)

if [[ $delete_to_line -ne $delete_from_line ]]
then
    delete_to_line=$(($delete_to_line - 1))
    sed -i "${delete_from_line},${delete_to_line}d" $cmake_file
fi

# delete controllers from aero_bringup.launch

launch_file="$(rospack find aero_description)/../aero_startup/aero_bringup.launch"
delete_from_launch=$(grep -n -m 1 ">>> add controllers" $launch_file | cut -d ':' -f1)
delete_from_launch=$(($delete_from_launch + 1))
delete_to_launch=$(grep -n -m 1 "<<< add controllers" $launch_file | cut -d ':' -f1)

if [[ $delete_to_launch -ne $delete_from_launch ]]
then
    delete_to_launch=$(($delete_to_launch - 1))
    sed -i "${delete_from_launch},${delete_to_launch}d" $launch_file
fi

# delete applications from CMakeLists.txt

delete_from_line=$(grep -n -m 1 ">>> add applications" $cmake_file | cut -d ':' -f1)
delete_from_line=$(($delete_from_line + 1))
delete_to_line=$(grep -n -m 1 "<<< add applications" $cmake_file | cut -d ':' -f1)

if [[ $delete_to_line -ne $delete_from_line ]]
then
    delete_to_line=$(($delete_to_line - 1))
    sed -i "${delete_from_line},${delete_to_line}d" $cmake_file
fi

# delete applications from applications.launch

launch_file="$(rospack find aero_description)/../aero_startup/applications.launch"
delete_from_launch=$(grep -n -m 1 ">>> add applications" $launch_file | cut -d ':' -f1)
delete_from_launch=$(($delete_from_launch + 1))
delete_to_launch=$(grep -n -m 1 "<<< add applications" $launch_file | cut -d ':' -f1)

if [[ $delete_to_launch -ne $delete_from_launch ]]
then
    delete_to_launch=$(($delete_to_launch - 1))
    sed -i "${delete_from_launch},${delete_to_launch}d" $launch_file
fi

# delete srv

num_of_srvs=$(grep ".srv" $cmake_file | wc -l)
delete_line=$(grep -n -m 1 "auto-add services" $cmake_file | cut -d ':' -f1)
delete_line=$(($delete_line + 3))
for (( num=1; num<=${num_of_srvs}; num++ ))
do
    sed -i "${delete_line}d" $cmake_file
done
sed -i "s/set(GENERATE_SRV 1)/set(GENERATE_SRV)/g" $cmake_file

# delete all auto-generated files

cd "$(rospack find aero_description)/../aero_startup"

delete_files=$(grep -r " \* This file auto-generated" | grep -v 'sed' | cut -d: -f1)
num_of_files=$(grep -r " \* This file auto-generated" | grep -v 'sed' | wc -l)
for (( num=1; num<=${num_of_files}; num++ ))
do
    file=$(echo $delete_files | awk '{print $'$num'}' | xargs rm)
done
