#!/bin/bash

# prerequisites : {my_robot}/models/gen/{my_upper_body}.l
# options(TBD) : generate {my_lower_body}.l, currently not supported

# generates : aero_description/models/{my_upper_body}.l
# generates : aero_description/models/{mesh_dir}/{mesh}.dae

dir=$1
upper_file=$2
lower_file=$3

# check state of models directory
models_dir_exists=$(ls $(rospack find aero_description) | grep models)

if [[ $models_dir_exists == '' ]]
then
    cp -r $(rospack find aero_description)/${dir}/models/urdf $(rospack find aero_description)/models
else
    rm -rf $(rospack find aero_description)/models
    cp -r $(rospack find aero_description)/${dir}/models/urdf $(rospack find aero_description)/models
fi

# copy eus file
cp $(rospack find aero_description)/${dir}/models/gen/${upper_file}.l $(rospack find aero_description)/models/

# add missings
add_missings() {
    meshes=$(awk '!seen[$0]++' /tmp/setupmodelaero)
    nums=$(echo $meshes | grep -o " " | wc -l)
    nums=$(($nums + 1))
    nums=$(($nums / 2))
    for (( mesh_number=1; mesh_number<=$nums; mesh_number++))
    do
	filename=$(echo $meshes | awk '{print $'$(($mesh_number * 2))'}')
	dirname=$(echo $meshes | awk '{print $'$(($mesh_number * 2 - 1))'}')
	has=$(find $(rospack find aero_description)/models/$dirname -name $filename)
	if [[ $has == '' ]]
	then
	    get=$(find $(rospack find aero_description)/ -name $filename | grep $dirname)
	    dir_exists=$(ls $(rospack find aero_description)/models | grep $dirname)
	    if [[ $dir_exists == '' ]]
	    then
		mkdir $(rospack find aero_description)/models/$dirname
	    fi
	    # echo "getting $filename"
	    cp $get $(rospack find aero_description)/models/$dirname/$filename
	fi
    done
}

# find missing meshes and add meshes
grep .dae $(rospack find aero_description)/models/${upper_file}.urdf | sed -e "s@filename=@%@g" | cut -d% -f2 | cut -d\" -f2 | awk -F/ '{print $(NF-1),$NF}' > /tmp/setupmodelaero
add_missings

grep .dae $(rospack find aero_description)/models/${lower_file}.urdf | sed -e "s@filename=@%@g" | cut -d% -f2 | cut -d\" -f2 | awk -F/ '{print $(NF-1),$NF}' > /tmp/setupmodelaero
add_missings

