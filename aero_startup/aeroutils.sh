# interpolation settings from command line
function intAero() {
    if [[ $1 == "" ]]
    then
        echo "usage: intAero [constant/linear/bezier/sigmoid/cbezier]"
        return
    fi

    type=0
    if [[ $1 == "constant" ]]
    then
        type=0
    elif [[ $1 == "linear" ]]
    then
        type=1
    elif [[ $1 == "bezier" ]]
    then
        type=2
    elif [[ $1 == "sigmoid" ]]
    then
        type=5
    elif [[ $1 == "cbezier" ]]
    then
        type=6
    fi
    rosservice call /aero_controller/interpolation [$type] []
}

# rotate kinect from command line
function rotateKinect() {
    if [[ $1 == "" ]]
    then
        echo "usage: rotateKinect [angle]"
        return
    fi

    rostopic pub --once /kinect_controller/command std_msgs/Float32 "{data: $1}"
}

# send speech from command line
function speakAero() {
    if [[ $1 == "" ]]
    then
        echo "usage: speakAero [notes]"
        return
    fi

    rostopic pub --once /windows/voice std_msgs/String "{data: $1}"
}
