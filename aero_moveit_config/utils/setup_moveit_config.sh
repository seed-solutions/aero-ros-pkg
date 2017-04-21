#!/bin/bash

if [[ $1 == "" ]]; then
    echo "if you want to make srdf, type like below"
    echo "./setup_moveit_config.sh [typeC, typeF]"
    exit
fi

srdf_template="$(rospack find aero_moveit_config)/config/AeroUpperRobot.template"
srdf_product="$(rospack find aero_moveit_config)/config/AeroUpperRobot.srdf"
collision_file="$(rospack find aero_moveit_config)/config/collision_$1.xml"


if [ -e $collision_file ]; then
    echo "creating AeroUpperRobot.srdf with collision file for $1"
else
    echo "collision_$1.xml not found"
    exit
fi



cp $srdf_template $srdf_product
collision_line=$(grep -n -m 1 "COLLISIONINSERTLINE" $srdf_product | cut -d ':' -f1)
cat $collision_file | sed -e 's/\//\\\//g; s/\"/\\\"/g' > /tmp/srdf_tmp
sed '1!G;h;$!d' /tmp/srdf_tmp | xargs -I{} sed -i "${collision_line}i\{}" $srdf_product
