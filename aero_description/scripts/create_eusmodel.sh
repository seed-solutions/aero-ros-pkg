#!/bin/bash

# prerequisites : {my_robot}/robots/aero.{urdf.xacro,yaml}

# generates : aero_description/robots/aero.{urdf,l}

aero_description_path=$(rospack find aero_description)
aero_xacro=${aero_description_path}/robots/aero.urdf.xacro
aero_yaml=${aero_description_path}/robots/aero.yaml
aero_urdf=${aero_description_path}/robots/aero.urdf
aero_l=${aero_description_path}/robots/aero.l

if [ -e ${aero_xacro} -a -e ${aero_yaml} ]
then
  rosrun xacro xacro.py ${aero_xacro} > ${aero_urdf}
  rosrun euscollada collada2eus_urdfmodel -I ${aero_urdf} -O ${aero_l} -C ${aero_yaml}
fi
