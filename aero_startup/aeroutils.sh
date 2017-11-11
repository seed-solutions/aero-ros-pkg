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

# change speed of kinect rotation from command line
function speedKinect() {
    if [[ $1 == "" ]]
    then
        echo "usage: speedKinect [speed 100~1000]"
        return
    fi

    rostopic pub --once /kinect_controller/speed std_msgs/Int32 "{data: $1}"
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

# save current aero's pose
function savePoseAero() {
    if [[ $1 == "" ]]
    then
        echo "usage: savePoseAero [name-to-save]"
        return
    fi

    name="["$1"]"
    (
        roscd aero_startup/poses

        if [ `cat pose_list.txt | grep -F $name` ]; then
            echo $1" already exists"
            return
        fi
        echo "new pose"

        file_name="$1.txt"
        rostopic echo -n 1 /aero_joint_states > $file_name
        sed -i '/\-\-\-/d' $file_name

        echo $name >> pose_list.txt
    )
}

function removePoseAero() {
    if [[ $1 == "" ]]
    then
        echo "usage: removePoseAero [name-to-remove]"
        return
    fi
    name="["$1"]"
    (
        roscd aero_startup/poses

        if [ -z `cat pose_list.txt | grep -F $name` ]; then
            echo "$1 is not found"
            return
        fi

        sed -i '/\['$1'\]/d' pose_list.txt
        rm "$1.txt"
    )
}

function loadPoseAero() {
    if [[ $1 == "" ]]
    then
        echo "usage: loadPoseAero [name-to-save]"
        return
    fi

    tsec=3
    if [[ $2 != "" ]]
    then
        tsec=$2
    fi

    name="["$1"]"
    (
        roscd aero_startup/poses

        if [ -z `cat pose_list.txt | grep -F $name` ]; then
            echo "$1 is not found"
            return
        fi

        file_name="$1.txt"

        jn=`cat $file_name | grep name | sed -e 's/name\://'`
        ps=`cat $file_name | grep position | sed -e 's/position\://'`
        rostopic pub --once /aero_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names: $jn
points:
- positions: $ps
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: $tsec, nsecs: 0}"

    )
}

function graspAero() {
    if [[ $1 == "" ]]
    then
        echo "usage: graspAero [right | rarm | left | larm]"
        return
    fi

    hand=$1
    if [[ $1 == "rarm" ]]
    then
        hand="right"
    fi
    if [[ $1 == "larm" ]]
    then
        hand="left"
    fi

    rosservice call /aero_hand_controller "{hand: '$hand', command: 'grasp', thre_fail: 0.0, thre_warn: 0.0, larm_angle: 0.0, rarm_angle: 0.0}"
}

function ungraspAero() {
    if [[ $1 == "" ]]
    then
        echo "usage: ungraspAero [right | rarm | left | larm]"
        return
    fi

    hand=$1
    if [[ $1 == "rarm" ]]
    then
        hand="right"
    fi
    if [[ $1 == "larm" ]]
    then
        hand="left"
    fi

    rosservice call /aero_hand_controller "{hand: '$hand', command: 'ungrasp', thre_fail: 0.0, thre_warn: 0.0, larm_angle: 0.0, rarm_angle: 0.0}"
}
