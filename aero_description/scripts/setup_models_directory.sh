#!/bin/bash

# prerequisites : {my_robot}/models/gen/{my_upper_body}.l
# options(TBD) : generate {my_lower_body}.l, currently not supported

# generates : aero_description/models/{my_upper_body}.l

dir=$1
upper_file=$2
lower_file=$3

# check state of models directory
if [[ ! -d $(rospack find aero_description)/models ]]
then
    mkdir $(rospack find aero_description)/models
else
    rm -rf $(rospack find aero_description)/models
    mkdir $(rospack find aero_description)/models
fi

# copy eus file
cp $(rospack find aero_description)/${dir}/models/gen/${upper_file}.l $(rospack find aero_description)/models/
