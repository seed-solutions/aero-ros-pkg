#!/bin/bash

names=()

create_go_pos_from_calib() {
    name=()
    angle=()
    vel=()
    time=()
    trans=()
    rot=()
    x=()
    y=()
    theta=()

    file="$(rospack find aero_ros_bridge)/config/${1}.yaml"
    j=0
    idx=-1
    while read line
    do
        var=$(echo "$line" | cut -d ':' -f1 | sed "{s/ $//}")
	val=$(echo "$line" | cut -d ':' -f2 | sed "{s/ //}")
	if [[ $var == "name" ]]
	then
	    idx=$(($idx + 1))
	    name[${idx}]="$val"
	    x[${idx}]="0"
	    y[${idx}]="0"
	    theta[${idx}]="0"
	elif [[ $var == "angle" ]]
	then
	    angle[${idx}]="$val"
	elif [[ $var == "vel" ]]
	then
	    vel[${idx}]="$val"
	elif [[ $var == "time" ]]
	then
	    time[${idx}]="$val"
	elif [[ $var == "trans" ]]
	then
	    trans[${idx}]="$val"
	elif [[ $var == "rot" ]]
	then
	    rot[${idx}]="$val"
	elif [[ $var == "allow" ]]
	then
	    if [[ $val == *"x"* ]] ; then
		x[${idx}]="1"
	    fi
	    if [[ $val == *"y"* ]] ; then
		y[${idx}]="1"
	    fi
	    if [[ $val == *"theta"* ]] ; then
		theta[${idx}]="1"
	    fi
	fi
        j=$(($j + 1))
    done < $file

    tab2=$'  '
    tab7=$'       '
    lisp=''
    idx=0

    for e in "${name[@]}"
    do
	lisp="${lisp}${tab2}(:$e (&optional (x 0) (y 0) (theta 0) (vel 308.0) (robot *aero*))\n"
	lisp="${lisp}${tab2}${tab2}(let (time-xy time-theta)\n"
	lisp="${lisp}${tab2}${tab2}${tab2}(if (and (eq x 0) (eq y 0) (eq theta 0))\n"
	lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(return-from :$e #f(${angle[$idx]})))\n"

	if [[ ${x[$idx]} == "1" ]] || [[ ${y[$idx]} == "1" ]]
	then
	    if [[ ${x[$idx]} == "0" ]] ; then
		lisp="${lisp}${tab2}${tab2}${tab2}(if (not (= x 0)) (warn \"x translation not allowed at this posture~%\"))\n"
		lisp="${lisp}${tab2}${tab2}${tab2}(setq x 0)\n"
	    fi
	    if [[ ${y[$idx]} == "0" ]] ; then
		lisp="${lisp}${tab2}${tab2}${tab2}(if (not (= y 0)) (warn \"y translation not allowed at this posture~%\"))\n"
		lisp="${lisp}${tab2}${tab2}${tab2}(setq y 0)\n"
	    fi
	    lisp="${lisp}${tab2}${tab2}${tab2}(setq time-xy (/ (* ${vel[$idx]} ${time[$idx]} (sqrt (+ (* x x) (* y y)))) (* vel ${trans[$idx]})))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}(if (not (= time-xy 0))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(progn (send self :wheel-vector (scale (/ vel (abs (+ x y))) (float-vector (- x y) (- (+ x y)) (+ x y) (- y x))))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(unix::usleep (round (* time-xy 1000)))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(send self :wheel-vector #f(0 0 0 0))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(return-from :$e (list x y 0))))\n"
	else
	    lisp="${lisp}${tab2}${tab2}${tab2}(if (or (not (= x 0) (= y 0))) (warn \"translation not allowed at this posture~%\"))\n"
	fi

	if [[ ${theta[$idx]} == "1" ]]
	then
	    lisp="${lisp}${tab2}${tab2}${tab2}(if (> (abs theta) 90) (progn (warn \"illegal theta input~%\") (setq theta 0)))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}(setq time-theta (abs (/ (* ${vel[$idx]} ${time[$idx]} theta) (* vel ${rot[$idx]}))))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}(if (not (= time-theta 0))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(progn (send self :wheel-vector (scale (/ theta (abs theta)) (float-vector vel (- vel) (- vel) vel)))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(unix::usleep (round (* time-theta 1000)))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(send self :wheel-vector #f(0 0 0 0))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(return-from :$e (list 0 0 theta))))\n"
	else
	    lisp="${lisp}${tab2}${tab2}${tab2}(if (not (= theta 0)) (warn \"rotation not allowed at this posture~%\"))\n"
	fi

	lisp="${lisp}${tab2}${tab2}${tab2}nil))\n"
	idx=$(($idx + 1))
	names[${#names[@]}]="$e"
    done

    echo "${names[@]}" > /tmp/go_pos_names

    echo -e "${lisp}"
}

body=''
tab2=$'  '
tab3=$'   '

body="${body}(defmethod aero-upper-interface\n"
body="${body}$(create_go_pos_from_calib go_pos_calib)\n"
#read -r -a names < /tmp/go_pos_names
#body="${body}$(create_go_pos_from_calib other_calibs)\n"
body="${body}${tab2}(:go-pos (x y theta &key (vel 308.0) (robot *aero*))\n"
body="${body}${tab2}${tab2}(cond\n"
read -r -a names < /tmp/go_pos_names
for e in "${names[@]}"
do
    body="${body}${tab2}${tab2}${tab2}((equal (send robot :lower :angle-vector) (send self :$e))\n"
    body="${body}${tab2}${tab2}${tab3}(send self :$e x y theta (float vel) robot))\n"
done
body="${body}${tab2}${tab2}))\n"
body="${body})\n"

echo -e "${body}" > "$(rospack find aero_ros_bridge)/euslisp/aero-go-pos.l"
