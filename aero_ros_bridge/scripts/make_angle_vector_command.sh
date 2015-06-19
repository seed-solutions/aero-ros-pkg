#!/bin/bash

# rsP,rsR,rsY,re,rwY,rwP,rwR,rt1,rf1,rf2,rf3,lsP,lsR,lsY,le,lwY,lwP,lwR,lt1,lf1,lf2,lf3,wY,wP,wR,nY,nP,nR
#                   ,rwT,rwB,   ,nil,nil,nil,                  ,lwT,lwB,   ,nil,nil,nil,  ,wR,wL,  ,nR,nL

create_table_func_from_csv() { # joint_name table/real_origin_diff
    # load csv
    csv=()
    csv_interval=()
    file="$(rospack find aero_ros_bridge)/models/${1}.csv"
    j=0
    while read line
    do
	csv[j]=$(echo "$line" | cut -d ',' -f4)
	csv_interval[j]=$(echo "$line" | cut -d ',' -f3)
	j=$(($j + 1))
    done < $file

    # preparation for stroke-angle-table : organize csv according to integral
    table=()
    ntable=()
    ntable[0]=""
    idx=0
    bef_head=""
    for e in "${csv[@]}"
    do
	val=$(echo "$2 + $e" | bc)
	head=$(echo "${val}" | sed s/\.[0-9,]*$//g)

	# getting rid of possible bugs
	if [[ $head == "-" ]]
	then
	    head=0
	elif [[ $head == "" ]]
	then
	    head=0
	fi

	if [[ $head -lt 0 ]] # when stroke is negative
	then
	    head=$(echo "-1 * $head" | bc)

	    # if +1 degree results to more than +1 stroke
	    if [[ ${bef_head} != ""  ]]
	    then
		if [[ $(echo "$head - $bef_head" | bc ) -gt 1 ]]
		then
		    ntable[$(($head - 1))]=""
		fi
	    fi
	    bef_head=$head

	    ntable[$head]="${ntable[$head]} (list ${idx} ${val} ${csv_interval[${idx}]})"
	else # when stroke is positive

	    # if +1 degree results to more than +1 stroke
	    if [[ ${bef_head} != ""  ]]
	    then
		if [[ $(echo "$head - $bef_head" | bc ) -gt 1 ]]
		then
		    table[$(($head - 1))]=""
		fi
	    fi
	    bef_head=$head

	    table[$head]="${table[$head]} (list ${idx} ${val} ${csv_interval[${idx}]})"
	fi
	idx=$(($idx + 1))
    done

    # create lisp method for stroke-angle-table   
    tab2=$'  '
    strokes=''
    lisp="${tab2}(:rev-${1}-table (stroke)\n"
    lisp="${lisp}${tab2}${tab2}(let ((integral (if (< stroke 0) (ceiling stroke) (floor stroke)))\n"
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab2}candidates\n"
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab2}appendix\n"
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab2}(result nil))\n"
    lisp="${lisp}${tab2}${tab2}${tab2}(case integral\n"

    # negative stroke value case
    idx=0
    for e in "${ntable[@]}"
    do
	if [[ $e != "" ]]
	then
	    if [[ $idx -lt $((${#ntable[@]} - 1)) ]]
	    then
		j=1
		if [[ ${ntable[$(($idx + $j))]} == "" ]]
		then
		    j=2
		fi
		lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(-${idx} (setq candidates (list${e})) (setq appendix (list${ntable[$(($idx + $j))]})))\n"
	    else
		lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(-${idx} (setq candidates (list${e})))\n"
	    fi
	fi
	idx=$(($idx + 1))
    done

    # positive stroke value case
    idx=0
    for e in "${table[@]}"
    do
	if [[ $e != "" ]]
	then
	    if [[ $idx -lt $((${#table[@]} - 1)) ]]
	    then
		j=1
		if [[ ${ntable[$(($idx + $j))]} == "" ]]
		then
		    j=2
		fi
		lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(${idx} (setq candidates (list${e})) (setq appendix (list${table[$(($idx + 1))]})))\n"
	    else
		lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(${idx} (setq candidates (list${e})))\n"
	    fi
	fi
	idx=$(($idx + 1))
    done

    tab6='      '
    tab8='        '
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(t (return-from :rev-${1}-table nil))\n"
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2})\n"
    lisp="${lisp}${tab6}(cond ((< stroke 0)\n"
    lisp="${lisp}${tab6}${tab8}(if (>= (length candidates) 2)\n"
    lisp="${lisp}${tab6}${tab8}${tab2}(if (< (cadr (car candidates)) (cadr (cadr candidates))) (setq candidates (reverse candidates))))\n"
    lisp="${lisp}${tab6}${tab8}(dolist (x candidates)\n"
    lisp="${lisp}${tab6}${tab8}${tab2}(if (>= stroke (cadr x)) (progn (setq result (- (car x) (/ (- (cadr x) stroke) (caddr x)))) (return))))\n"
    lisp="${lisp}${tab6}${tab8}(if (eq result nil)\n"
    lisp="${lisp}${tab6}${tab8}${tab2}(progn (if (>= (length appendix) 2)\n"
    lisp="${lisp}${tab6}${tab8}${tab2}${tab8}(if (< (cadr (car appendix)) (cadr (cadr appendix))) (setq appendix (reverse appendix))))\n"
    lisp="${lisp}${tab6}${tab8}${tab2}${tab6}(if (eq appendix nil) (setq result (car (car (reverse candidates))))\n"
    lisp="${lisp}${tab6}${tab8}${tab2}${tab8}(setq result (- (car (car appendix)) (/ (- (cadr (car appendix)) stroke) (caddr (car appendix))))))))\n"
    lisp="${lisp}${tab6}${tab8})\n"

    lisp="${lisp}${tab6}${tab6}((>= stroke 0)\n"
    lisp="${lisp}${tab6}${tab8}(if (>= (length candidates) 2)\n"
    lisp="${lisp}${tab6}${tab8}${tab2}(if (> (cadr (car candidates)) (cadr (cadr candidates))) (setq candidates (reverse candidates))))\n"
    lisp="${lisp}${tab6}${tab8}(dolist (x candidates)\n"
    lisp="${lisp}${tab6}${tab8}${tab2}(if (<= stroke (cadr x)) (progn (setq result (- (car x) (if (eq (caddr x) 0) 0 (/ (- (cadr x) stroke) (caddr x))))) (return))))\n"
    lisp="${lisp}${tab6}${tab8}(if (eq result nil)\n"
    lisp="${lisp}${tab6}${tab8}${tab2}(progn (if (>= (length appendix) 2)\n"
    lisp="${lisp}${tab6}${tab8}${tab2}${tab8}(if (> (cadr (car appendix)) (cadr (cadr appendix))) (setq appendix (reverse appendix))))\n"
    lisp="${lisp}${tab6}${tab8}${tab2}${tab6}(if (eq appendix nil) (setq result (car (car (reverse candidates))))\n"
    lisp="${lisp}${tab6}${tab8}${tab2}${tab8}(setq result (- (car (car appendix)) (if (eq (caddr appendix) 0) 0 (/ (- (cadr (car appendix)) stroke) (caddr (car appendix)))))))))\n"
    lisp="${lisp}${tab6}))\n"
    lisp="${lisp}${tab2}${tab2}result))\n"

    echo -e "${lisp}"
}

body=''
tab2=$'  '
tab4=$'    '
tab6=$'      '
tab14=$'              '

body="${body}(defmethod aero-upper-interface\n"
body="${body}${tab2}(:stroke-to-angle (stroke)\n"
body="${body}${tab2}${tab2}(let ((neck-stroke-A (cdr (assoc \"neck_right_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab2}${tab6}(neck-stroke-B (cdr (assoc \"neck_left_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab2}${tab6}(waist-stroke-A (cdr (assoc \"waist_right_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab2}${tab6}(waist-stroke-B (cdr (assoc \"waist_left_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab2}${tab6}neck-pitch-stroke\n"
body="${body}${tab2}${tab2}${tab6}neck-roll-stroke\n"
body="${body}${tab2}${tab2}${tab6}waist-pitch-stroke\n"
body="${body}${tab2}${tab2}${tab6}waist-roll-stroke\n"
body="${body}${tab2}${tab2}${tab6}(neck-roll-direction 1)\n"
body="${body}${tab2}${tab2}${tab6}(waist-roll-direction 1)\n"
body="${body}${tab2}${tab2}${tab6}(left-wrist-stroke-A (cdr (assoc \"l_wrist_top_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab2}${tab6}(left-wrist-stroke-B (cdr (assoc \"l_wrist_bottom_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab2}${tab6}left-wrist-roll-stroke\n"
body="${body}${tab2}${tab2}${tab6}left-wrist-pitch-stroke\n"
body="${body}${tab2}${tab2}${tab6}(right-wrist-stroke-A (cdr (assoc \"r_wrist_top_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab2}${tab6}(right-wrist-stroke-B (cdr (assoc \"r_wrist_bottom_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab2}${tab6}right-wrist-roll-stroke\n"
body="${body}${tab2}${tab2}${tab6}right-wrist-pitch-stroke\n"
body="${body}${tab2}${tab2}${tab6}(left-wrist-pitch-direction 1)\n"
body="${body}${tab2}${tab2}${tab6}(left-wrist-roll-direction 1)\n"
body="${body}${tab2}${tab2}${tab6}(right-wrist-pitch-direction 1)\n"
body="${body}${tab2}${tab2}${tab6}(right-wrist-roll-direction 1)\n"
body="${body}${tab2}${tab2}${tab6})\n"
body="${body}${tab2}${tab6}(setq neck-pitch-stroke (/ (+ neck-stroke-A neck-stroke-B) 2))\n"
body="${body}${tab2}${tab6}(setq neck-roll-stroke (- neck-stroke-A neck-pitch-stroke))\n"
body="${body}${tab2}${tab6}(setq waist-pitch-stroke (/ (+ waist-stroke-A waist-stroke-B) 2))\n"
body="${body}${tab2}${tab6}(setq waist-roll-stroke (- waist-stroke-A waist-pitch-stroke))\n"
body="${body}${tab2}${tab6}(setq left-wrist-roll-stroke (/ (+ left-wrist-stroke-A left-wrist-stroke-B) 2))\n"
body="${body}${tab2}${tab6}(setq left-wrist-pitch-stroke (- left-wrist-stroke-A left-wrist-roll-stroke))\n"
body="${body}${tab2}${tab6}(setq right-wrist-roll-stroke (/ (+ right-wrist-stroke-A right-wrist-stroke-B) 2))\n"
body="${body}${tab2}${tab6}(setq right-wrist-pitch-stroke (- right-wrist-stroke-A right-wrist-roll-stroke))\n"
body="${body}${tab2}${tab6}(if (> neck-stroke-A neck-stroke-B)\n"
body="${body}${tab2}${tab6}${tab4}(setq neck-roll-direction -1) (setq neck-roll-direction 1))\n"
body="${body}${tab2}${tab6}(if (> waist-stroke-A waist-stroke-B)\n"
body="${body}${tab2}${tab6}${tab4}(setq wasit-roll-direction 1) (setq waist-roll-direction -1))\n"
body="${body}${tab2}${tab6}(if (> left-wrist-stroke-A left-wrist-stroke-B)\n"
body="${body}${tab2}${tab6}${tab4}(setq left-wrist-pitch-direction 1) (setq left-wrist-pitch-direction -1))\n"
body="${body}${tab2}${tab6}(if (>= left-wrist-roll-stroke 0)\n"
body="${body}${tab2}${tab6}${tab4}(setq left-wrist-roll-direction -1) (setq left-wrist-roll-direction 1))\n"
body="${body}${tab2}${tab6}(if (> right-wrist-stroke-A right-wrist-stroke-B)\n"
body="${body}${tab2}${tab6}${tab4}(setq right-wrist-pitch-direction 1) (setq right-wrist-pitch-direction -1))\n"
body="${body}${tab2}${tab6}(if (>= right-wrist-roll-stroke 0)\n"
body="${body}${tab2}${tab6}${tab4}(setq right-wrist-roll-direction 1) (setq right-wrist-roll-direction -1))\n"
body="${body}${tab2}${tab6}(list\n"

body="${body}${tab2}${tab6}(float-vector (- (send self :rev-shoulder-p-table (cdr (assoc \"r_shoulder_pitch_joint\" stroke :test #'equal))))\n"
body="${body}${tab2}${tab6}${tab14}(- (send self :rev-shoulder-r-table (cdr (assoc \"r_shoulder_roll_joint\" stroke :test #'equal))))\n"
body="${body}${tab2}${tab6}${tab14}(- (cdr (assoc \"r_elbow_yaw_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab6}${tab14}(- (send self :rev-elbow-p-table (cdr (assoc \"r_elbow_pitch_joint\" stroke :test #'equal))))\n"
body="${body}${tab2}${tab6}${tab14}(- (cdr (assoc \"r_wrist_roll_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab6}${tab14}(* right-wrist-pitch-direction (send self :rev-wrist-p-table (abs right-wrist-pitch-stroke)))\n"
body="${body}${tab2}${tab6}${tab14}(* right-wrist-roll-direction (send self :rev-wrist-r-table (- (abs right-wrist-roll-stroke))))\n"
body="${body}${tab2}${tab6}${tab14}(send self :rev-hand-table (cdr (assoc \"r_hand_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab6}${tab14}0\n"
body="${body}${tab2}${tab6}${tab14}0\n"
body="${body}${tab2}${tab6}${tab14}0\n"
body="${body}${tab2}${tab6}${tab14}(- (send self :rev-shoulder-p-table (cdr (assoc \"l_shoulder_pitch_joint\" stroke :test #'equal))))\n"
body="${body}${tab2}${tab6}${tab14}(send self :rev-shoulder-r-table (cdr (assoc \"l_shoulder_roll_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab6}${tab14}(- (cdr (assoc \"l_elbow_yaw_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab6}${tab14}(- (send self :rev-elbow-p-table (cdr (assoc \"l_elbow_pitch_joint\" stroke :test #'equal))))\n"
body="${body}${tab2}${tab6}${tab14}(- (cdr (assoc \"l_wrist_roll_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab6}${tab14}(* left-wrist-pitch-direction (send self :rev-wrist-p-table (abs left-wrist-pitch-stroke)))\n"
body="${body}${tab2}${tab6}${tab14}(* left-wrist-roll-direction (send self :rev-wrist-r-table (- (abs left-wrist-roll-stroke))))\n"
body="${body}${tab2}${tab6}${tab14}(- (send self :rev-hand-table (cdr (assoc \"l_hand_joint\" stroke :test #'equal))))\n"
body="${body}${tab2}${tab6}${tab14}0\n"
body="${body}${tab2}${tab6}${tab14}0\n"
body="${body}${tab2}${tab6}${tab14}0\n"
body="${body}${tab2}${tab6}${tab14}(cdr (assoc \"waist_pitch_joint\" stroke :test #'equal))\n"
body="${body}${tab2}${tab6}${tab14}(send self :rev-waist-p-table waist-pitch-stroke)\n"
body="${body}${tab2}${tab6}${tab14}(* waist-roll-direction (send self :rev-waist-r-table (abs waist-roll-stroke)))\n"
body="${body}${tab2}${tab6}${tab14}(cdr (assoc \"neck_yaw_joint\" stroke :test #'equal))\n"
body="${body}${tab2}${tab6}${tab14}(send self :rev-neck-p-table neck-pitch-stroke)\n"
body="${body}${tab2}${tab6}${tab14}(* neck-roll-direction (send self :rev-neck-r-table (abs neck-roll-stroke)))\n"
body="${body}${tab2}${tab6}${tab14})\n"

body="${body}${tab2}${tab6}(float-vector (- (cdr (assoc \"r_r_crotch_yaw_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab6}${tab14}(send self :rev-crotch-p-table (cdr (assoc \"r_r_crotch_pitch_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab6}${tab14}(send self :rev-knee-p-table (cdr (assoc \"r_r_knee_pitch_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab6}${tab14}(- (cdr (assoc \"f_r_crotch_yaw_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab6}${tab14}(send self :rev-crotch-p-table (cdr (assoc \"f_r_crotch_pitch_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab6}${tab14}(send self :rev-knee-p-table (cdr (assoc \"f_r_knee_pitch_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab6}${tab14}(- (cdr (assoc \"r_l_crotch_yaw_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab6}${tab14}(send self :rev-crotch-p-table (cdr (assoc \"r_l_crotch_pitch_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab6}${tab14}(send self :rev-knee-p-table (cdr (assoc \"r_l_knee_pitch_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab6}${tab14}(- (cdr (assoc \"f_l_crotch_yaw_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab6}${tab14}(send self :rev-crotch-p-table (cdr (assoc \"f_l_crotch_pitch_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab6}${tab14}(send self :rev-knee-p-table (cdr (assoc \"f_l_knee_pitch_joint\" stroke :test #'equal)))\n"
body="${body}${tab2}${tab6}${tab14}0\n"
body="${body}${tab2}${tab6}${tab14})\n"

body="${body}${tab2}${tab6}${tab14})))\n"
body="${body}$(create_table_func_from_csv shoulder-p 0)\n"
body="${body}$(create_table_func_from_csv shoulder-r 0)\n"
body="${body}$(create_table_func_from_csv elbow-p 20.183)\n"
body="${body}$(create_table_func_from_csv wrist-p 0)\n"
body="${body}$(create_table_func_from_csv wrist-r 0)\n"
body="${body}$(create_table_func_from_csv waist-p 0)\n"
body="${body}$(create_table_func_from_csv waist-r 0)\n"
body="${body}$(create_table_func_from_csv neck-p 0)\n"
body="${body}$(create_table_func_from_csv neck-r 0)\n"

body="${body}$(create_table_func_from_csv crotch-p 0)\n"
body="${body}$(create_table_func_from_csv knee-p 0)\n"
body="${body})\n"

echo -e "${body}" > "$(rospack find aero_ros_bridge)/euslisp/aero-upper-lower-angles.l"
