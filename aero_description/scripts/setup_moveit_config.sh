#!/bin/bash

# prerequisites : {my_robot}/moveit_config

# cp {my_robot}/moveit_config to aero_moveit_config and compile the pkg

robot=$1

if [[ -e ${robot}/moveit_config ]]
then
    if [[ -e ${robot}/moveit_config/config ]]
    then
        echo "copy to aero_moveit_config/config"
        cp ${robot}/moveit_config/config/* ../aero_moveit_config/config/
    fi

    if [[ -e ${robot}/moveit_config/launch ]]
    then
        echo "copy to aero_moveit_config/launch"
        cp ${robot}/moveit_config/launch/* ../aero_moveit_config/launch/
    fi

else
    echo "directory: "${robot}"/moveit_config is not found"
fi

catkin build aero_moveit_config
