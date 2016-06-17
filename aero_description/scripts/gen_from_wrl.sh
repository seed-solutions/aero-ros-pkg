#!/bin/bash

# prerequisites : {my_robot}/models/wrl

# generates : {my_robot}/models/gen

dir=$1
file=$2

cmake_file=$(rospack find aero_description)/CMakeLists.txt
gen_dir=$(rospack find aero_description)/$dir/models/gen
urdf_dir=$(rospack find aero_description)/$dir/models/urdf
backup_file=/tmp/aeroDescripCMake
backup_gen=/tmp/aeroModelsGen
backup_urdf=/tmp/aeroModelsUrdf

if [[ $gen_dir = "" ]]
then
    echo "aero_description not found"
    exit 1
fi

if [[ $file = "" ]]
then
    file=aero_upper
fi

cp $cmake_file $backup_file
cp -r $gen_dir $backup_gen
cp -r $urdf_dir $backup_urdf

sed -i "s@# @@g" $cmake_file
sed -i "s@find_package(catkin REQUIRED)@# find_package(catkin REQUIRED)@g" $cmake_file
sed -i "s@/models/aero_upper.wrl@/models/$file.wrl@g" $cmake_file

models_dir=$(rospack find aero_description)/models
if [[ -d $models_dir ]]
then
    rm -rf $models_dir
fi
mkdir $models_dir

cp $(rospack find aero_description)/$dir/models/wrl/*.wrl $models_dir/
cp $(rospack find aero_description)/$dir/models/wrl/$file.yaml $models_dir/

catkin b aero_description

cp $backup_file $cmake_file
if [[ -d $urdf_dir/${file}_meshes ]]
then
    echo "removing meshes"
    rm -rf $urdf_dir/${file}_meshes
fi
mv $models_dir/${file}_meshes $urdf_dir/
cp $models_dir/* $gen_dir/
rm $gen_dir/*.wrl
rm $gen_dir/$file.yaml
rm -rf $models_dir
