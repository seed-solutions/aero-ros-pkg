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
	lisp="${lisp}${tab2}(:$e (&optional (x 0) (y 0) (theta 0) (exec-large nil) (vel 308.0) (robot *aero*) (sleep-step 200) (vel-step 60.0))\n"
	lisp="${lisp}${tab2}${tab2}(let (time-xy time-theta ui (~warn~ 1))\n"
	lisp="${lisp}${tab2}${tab2}${tab2}(if (and (eq x 0) (eq y 0) (eq theta 0))\n"
	lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(return-from :$e #f(${angle[$idx]})))\n"

	if [[ ${x[$idx]} == "1" ]] || [[ ${y[$idx]} == "1" ]]
	then
	    if [[ ${x[$idx]} == "0" ]] ; then
#		lisp="${lisp}${tab2}${tab2}${tab2}(if (not (= x 0)) (warn \"x translation not allowed at this posture~%\"))\n"
		lisp="${lisp}${tab2}${tab2}${tab2}(if (not (= x 0)) (setq ~warn~ -4))\n"
		lisp="${lisp}${tab2}${tab2}${tab2}(setq x 0)\n"
	    fi
	    if [[ ${y[$idx]} == "0" ]] ; then
#		lisp="${lisp}${tab2}${tab2}${tab2}(if (not (= y 0)) (warn \"y translation not allowed at this posture~%\"))\n"
		lisp="${lisp}${tab2}${tab2}${tab2}(if (not (= y 0)) (setq ~warn~ -4))\n"
		lisp="${lisp}${tab2}${tab2}${tab2}(setq y 0)\n"
	    fi
	    lisp="${lisp}${tab2}${tab2}${tab2}(setq time-xy (/ (* ${vel[$idx]} ${time[$idx]} (sqrt (+ (* x x) (* y y)))) (* vel ${trans[$idx]})))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}(if (not (= time-xy 0))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(progn (cond ((or (> x 1000) (> y 1000))\n"
#	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}${tab7}(print \"large input value! continue?\")\n"
#	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}${tab7}(setq ui (read-line))\n"
#	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}${tab7}(if (or (equal ui \"no\") (equal ui \"n\") (equal ui \"nil\")) (return-from :$e (warn \"cancelled~%\")))))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}${tab7}(if (eq exec-large nil) (return-from :$e -3))))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(let ((time-xy-left time-xy)  sleep-time (current-vel 0.0))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(when *real* (do-until-key\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(when (< time-xy-left 0.0) (return nil))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(setq current-vel (min (+ current-vel vel-step) vel))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(setq sleep-time (if (< time-xy-left sleep-step) (round time-xy-left) sleep-step))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(setq time-xy-left (- time-xy-left sleep-step))\n"

	    #lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(send self :wheel-vector (scale (/ vel (abs (+ x y))) (float-vector (- x y) (- (+ x y)) (+ x y) (- y x))))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(send self :wheel-vector (scale (/ current-vel (abs (+ x y))) (float-vector (- x y) (- (+ x y)) (+ x y) (- y x))))\n"

	    #lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(if (eq *real* t) (unix::usleep (round (* time-xy 1000))))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(unix::usleep (* sleep-time 1000)))\n"

	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(do-until-key\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(when (< current-vel 1.0) (return nil))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(setq current-vel (max (- current-vel vel-step) 0.0))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(send self :wheel-vector (scale (/ current-vel (abs (+ x y))) (float-vector (- x y) (- (+ x y)) (+ x y) (- y x))))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(unix::usleep (* sleep-step 1000)))\n"

	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(send self :wheel-vector #f(0 0 0 0)))\n"
#	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(return-from :$e (list x y 0))))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(return-from :$e ~warn~))))\n"
	else
#	    lisp="${lisp}${tab2}${tab2}${tab2}(if (or (not (= x 0) (= y 0))) (warn \"translation not allowed at this posture~%\"))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}(if (or (not (= x 0) (= y 0))) (setq ~warn~ -4))\n"
	fi

	if [[ ${theta[$idx]} == "1" ]]
	then
#	    lisp="${lisp}${tab2}${tab2}${tab2}(if (> (abs theta) 90) (progn (warn \"illegal theta input~%\") (setq theta 0)))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}(if (> (abs theta) 90) (progn (setq ~warn~ -3) (setq theta 0)))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}(setq time-theta (abs (/ (* ${vel[$idx]} ${time[$idx]} theta) (* vel ${rot[$idx]}))))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}(if (not (= time-theta 0))\n"
#	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(progn (send self :wheel-vector (scale (/ theta (abs theta)) (float-vector vel (- vel) (- vel) vel)))\n"
#	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(if (eq *real* t) (unix::usleep (round (* time-theta 1000))))\n"

	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(progn\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(let ((time-theta-left time-theta) sleep-time (current-vel 0.0))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(when *real* (do-until-key\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(when (< time-theta-left 0.0) (return nil))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(setq current-vel (min (+ current-vel vel-step) vel))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(setq sleep-time (if (< time-theta-left sleep-step) (round time-theta-left) sleep-step))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(setq time-theta-left (- time-theta-left sleep-step))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(send self :wheel-vector (scale (/ theta (abs theta)) (float-vector current-vel (- current-vel) (- current-vel) current-vel)))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(unix::usleep (* sleep-time 1000))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(setq time-theta-left (- time-theta-left sleep-step)))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(do-until-key\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(when (< current-vel 1.0) (return nil))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(setq current-vel (max (- current-vel vel-step) 0.0))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(send self :wheel-vector (scale (/ theta (abs theta)) (float-vector current-vel (- current-vel) (- current-vel) current-vel)))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(unix::usleep (* sleep-step 1000)))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(send self :wheel-vector #f(0 0 0 0)))\n"
#	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(return-from :$e (list 0 0 theta))))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab7}(return-from :$e ~warn~))))\n"
	else
#	    lisp="${lisp}${tab2}${tab2}${tab2}(if (not (= theta 0)) (warn \"rotation not allowed at this posture~%\"))\n"
	    lisp="${lisp}${tab2}${tab2}${tab2}(if (not (= theta 0)) (setq ~warn~ -4))\n"
	fi

#	lisp="${lisp}${tab2}${tab2}${tab2}nil))\n"
	lisp="${lisp}${tab2}${tab2}${tab2}~warn~))\n"
	idx=$(($idx + 1))
	if [[ $e != "gp-nil" ]]
	then
	    names[${#names[@]}]="$e"
	fi
    done

    echo "${names[@]}" > /tmp/go_pos_names

    echo -e "${lisp}"
}

body=''
tab2=$'  '
tab3=$'   '

body="${body}(defvar *wheel-velocity* 308.0)\n\n"
body="${body}(defmethod aero-upper-interface\n"
body="${body}$(create_go_pos_from_calib go_pos_calib)\n"
#read -r -a names < /tmp/go_pos_names
#body="${body}$(create_go_pos_from_calib other_calibs)\n"
body="${body}${tab2}(:go-pos (x y theta &optional (exec-large nil) (vel *wheel-velocity*) (robot *aero*) (sleep-step 200) (vel-step 60.0))\n"
body="${body}${tab2}${tab2}(let (res ui)\n"
body="${body}${tab2}${tab2}${tab2}(if (eq *real* t) (send self :wheel-on))\n"
body="${body}${tab2}${tab2}${tab2}(unix:usleep (* 1000 300))\n"
body="${body}${tab2}${tab2}${tab2}(cond\n"
read -r -a names < /tmp/go_pos_names
for e in "${names[@]}"
do
    body="${body}${tab2}${tab2}${tab2}${tab2}((equal (send robot :lower :angle-vector) (send self :$e))\n"
    body="${body}${tab2}${tab2}${tab2}${tab3}(setq res (send self :$e x y theta exec-large (float vel) robot sleep-step vel-step)))\n"
done

body="${body}${tab2}${tab2}${tab2}${tab2}(t\n"
#body="${body}${tab2}${tab2}${tab2}${tab3}(print \"not a registered pose! continue?\")\n"
#body="${body}${tab2}${tab2}${tab2}${tab3}(setq ui (read-line))\n"
#body="${body}${tab2}${tab2}${tab2}${tab3}(if (or (equal ui \"no\") (equal ui \"n\") (equal ui \"nil\"))\n"
#body="${body}${tab2}${tab2}${tab2}${tab3}${tab3}(progn (send self :wheel-off) (return-from :go-pos (warn \"cancelled~%\"))))\n"
body="${body}${tab2}${tab2}${tab2}${tab3}(setq res (send self :gp-nil x y theta exec-large (float vel) robot sleep-step vel-step)))\n"
body="${body}${tab2}${tab2}${tab2})\n"
body="${body}${tab2}${tab2}${tab2}(unix:usleep (* 1000 300))\n"
body="${body}${tab2}${tab2}${tab2}(if (eq *real* t) (send self :wheel-off))\n"
body="${body}${tab2}${tab2}res))\n"
body="${body})\n"

echo -e "${body}" > "$(rospack find aero_ros_bridge)/euslisp/aero-go-pos.l"
