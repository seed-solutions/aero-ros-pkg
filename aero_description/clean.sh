#!/bin/bash

# delete CMakeLists.txt

rm "$(rospack find aero_description)/../aero_startup/CMakeLists.txt"

# delete controllers from aero_bringup.launch

rm "$(rospack find aero_description)/../aero_startup/generated_controllers.launch"

# delete all auto-generated files

cd "$(rospack find aero_description)/../aero_startup"

delete_files=$(grep -r " \* This file auto-generated" | grep -v 'sed' | cut -d: -f1)
num_of_files=$(grep -r " \* This file auto-generated" | grep -v 'sed' | wc -l)
for (( num=1; num<=${num_of_files}; num++ ))
do
    file=$(echo $delete_files | awk '{print $'$num'}' | xargs rm)
done
