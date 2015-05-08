#!/bin/bash

# rsP,rsR,rsY,re,rwY,rwP,rwR,rt1,rf1,rf2,rf3,lsP,lsR,lsY,le,lwY,lwP,lwR,lt1,lf1,lf2,lf3,wY,wP,wR,nY,nP,nR
#                   ,rwT,rwB,   ,nil,nil,nil,                  ,lwT,lwB,   ,nil,nil,nil,  ,wR,wL,  ,nR,nL

create_table_func_from_csv() { # joint_name table/real_origin_diff
    # load csv
    table=()
    interval=()
    file="$(rospack find aero_ros_bridge)/models/${1}.csv"
    j=0
    while read line
    do
	table[j]=$(echo "$line" | cut -d ',' -f4)
	interval[j]=$(echo "$line" | cut -d ',' -f3)
	j=$(($j + 1))
    done < $file

    # create lisp method for angle-stroke-table
    tab2=$'  '
    strokes=''
    lisp="${tab2}(:${1}-table (angle)\n"
    lisp="${lisp}${tab2}${tab2}(let ((integral (ceiling angle))\n"
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab2}stroke\n"
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab2}interval)\n"
    lisp="${lisp}${tab2}${tab2}${tab2}(case integral\n"

    idx=0
    for e in "${table[@]}"
    do
	val=$(echo "$2 + $e" | bc)
	lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(${idx} (setq stroke ${val}) (setq interval ${interval[${idx}]}))\n"
	idx=$(($idx + 1))
    done
    
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(t (return-from :${1}-table nil))\n"
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2})\n"
    lisp="${lisp}${tab2}${tab2}${tab2}(- stroke (* interval (- integral angle))))\n"
    lisp="${lisp}${tab2}${tab2})\n"

    echo -e "${lisp}"
}

create_rp_converter_func_from_csv() { # joint_name roll-file pitch-file +/- +/- 0/1
    # load roll csv
    table_r=()
    interval_r=()
    file="$(rospack find aero_ros_bridge)/models/${2}.csv"
    j=0
    while read line
    do
	table_r[j]=$(echo "$line" | cut -d ',' -f4)
	interval_r[j]=$(echo "$line" | cut -d ',' -f3)
	j=$(($j + 1))
    done < $file
    # load pitch csv
    table_p=()
    interval_p=()
    file="$(rospack find aero_ros_bridge)/models/${3}.csv"
    j=0
    while read line
    do
	table_p[j]=$(echo "$line" | cut -d ',' -f4)
	interval_p[j]=$(echo "$line" | cut -d ',' -f3)
	j=$(($j + 1))
    done < $file

    # create lisp method for angle-stroke-table
    tab2=$'  '
    lisp="${tab2}(:${1}-rp-table (angle-r angle-p)\n"
    lisp="${lisp}${tab2}${tab2}(let ((roll-stroke nil) (pitch-stroke nil)\n"
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(integral-r (if (< angle-r 0) (floor angle-r) (ceiling angle-r)))\n"
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(integral-p (if (< angle-p 0) (floor angle-p) (ceiling angle-p)))\n"
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}stroke-r interval-r stroke-p interval-p)\n"

    strokes=''
    idx=0
    lisp="${lisp}${tab2}${tab2}${tab2}(case integral-r\n"
    for e in "${table_r[@]}"
    do
	if [[ ${idx} -ne 0 ]]
	then
	    val=$(echo "-1 * $e" | bc)
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(-${idx} (setq stroke-r ${val}) (setq interval-r ${interval_r[${idx}]}))\n"
	fi
	lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(${idx} (setq stroke-r ${e}) (setq interval-r ${interval_r[${idx}]}))\n"
	strokes="${strokes} ${e}"
	idx=$(($idx + 1))
    done
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(t (return-from :${1}-rp-table nil))\n"
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2})\n"
    lisp="${lisp}${tab2}${tab2}${tab2}(setq roll-stroke (- stroke-r (* interval-r (- integral-r angle-r))))\n"

    strokes=''
    idx=0
    lisp="${lisp}${tab2}${tab2}${tab2}(case integral-p\n"
    for e in "${table_p[@]}"
    do
	if [[ ${idx} -ne 0 ]]
	then
	    if [[ ${6} -eq 1 ]]
	    then
		val=$(echo "-1 * $e" | bc)
		lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(-${idx} (setq stroke-p ${val}) (setq interval-p ${interval_p[${idx}]}))\n"
	    fi
	fi
	lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(${idx} (setq stroke-p ${e}) (setq interval-p ${interval_p[${idx}]}))\n"
	strokes="${strokes} ${e}"
	idx=$(($idx + 1))
    done
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(t (return-from :${1}-rp-table nil))\n"
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2})\n"
    lisp="${lisp}${tab2}${tab2}${tab2}(setq pitch-stroke (- stroke-p (* interval-p (- integral-p angle-p))))\n"

    # calculate result in lisp
    lisp="${lisp}${tab2}${tab2}${tab2}(float-vector (${4} pitch-stroke roll-stroke) (${5} pitch-stroke roll-stroke)))\n"
    lisp="${lisp}${tab2}${tab2})\n"

    echo -e "${lisp}"
}


body=''
tab2=$'  '

body="${body}(defmethod AeroUpperRobot-robot\n"
body="${body}${tab2}(:stroke-vector ()\n"
#body="${body}${tab2}${tab2}(let (av result)\n"
body="${body}${tab2}${tab2}(let (av right-wrist-tb left-wrist-tb waist-rl neck-rl result)\n"
body="${body}${tab2}${tab2}${tab2}(setq av (send self :angle-vector))\n"
body="${body}${tab2}${tab2}${tab2}(setq right-wrist-tb (send self :wrist-rp-table (- (elt av 5)) (elt av 6)))\n"
body="${body}${tab2}${tab2}${tab2}(setq left-wrist-tb (send self :wrist-rp-table (- (elt av 16)) (elt av 17)))\n"
body="${body}${tab2}${tab2}${tab2}(setq waist-rl (send self :waist-rp-table (- (elt av 24)) (elt av 23)))\n"
body="${body}${tab2}${tab2}${tab2}(setq neck-rl (send self :neck-rp-table (- (elt av 27)) (elt av 26)))\n"
body="${body}${tab2}${tab2}${tab2}(setq result (float-vector\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :shoulder-p-table (- (elt av 0)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :shoulder-r-table (- (elt av 1)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 2)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :elbow-p-table (- (elt av 3)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 4)\n"
#body="${body}${tab2}${tab2}${tab2}${tab2}(send self :wrist-p-table (- (round (elt av 5))))\n"
#body="${body}${tab2}${tab2}${tab2}${tab2}(send self :wrist-r-table (round (elt av 6)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt right-wrist-tb 0)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt right-wrist-tb 1)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 7)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 8)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 9)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 10)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :shoulder-p-table (- (elt av 11)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :shoulder-r-table (elt av 12))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 13)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :elbow-p-table (- (elt av 14)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 15)\n"
#body="${body}${tab2}${tab2}${tab2}${tab2}(send self :wrist-p-table (- (round (elt av 16))))\n"
#body="${body}${tab2}${tab2}${tab2}${tab2}(send self :wrist-r-table (round (elt av 17)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt left-wrist-tb 0)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt left-wrist-tb 1)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 18)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 19)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 20)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 21)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 22)\n"
#body="${body}${tab2}${tab2}${tab2}${tab2}(send self :waist-p-table (round (elt av 23)))\n"
#body="${body}${tab2}${tab2}${tab2}${tab2}(send self :waist-r-table (- (round (elt av 24))))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt waist-rl 0)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt waist-rl 1)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 25)\n"
#body="${body}${tab2}${tab2}${tab2}${tab2}(send self :neck-p-table (round (elt av 26)))\n"
#body="${body}${tab2}${tab2}${tab2}${tab2}(send self :neck-r-table (- (round (elt av 27))))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt neck-rl 0)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt neck-rl 1)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}))\n"
body="${body}${tab2}${tab2}${tab2}result))\n"
body="${body}$(create_table_func_from_csv shoulder-p 0)\n"
body="${body}$(create_table_func_from_csv shoulder-r 0)\n"
body="${body}$(create_table_func_from_csv elbow-p 20.183)\n"
#body="${body}$(create_table_func_from_csv wrist-r 0)\n"
#body="${body}$(create_table_func_from_csv wrist-p 0)\n"
#body="${body}$(create_table_func_from_csv waist-p 0)\n"
#body="${body}$(create_table_func_from_csv waist-r 0)\n"
#body="${body}$(create_table_func_from_csv neck-p 0)\n"
#body="${body}$(create_table_func_from_csv neck-r 0)\n"
body="${body}$(create_rp_converter_func_from_csv wrist wrist-p wrist-r + - 1)\n"
body="${body}$(create_rp_converter_func_from_csv waist waist-r waist-p - + 0)\n"
body="${body}$(create_rp_converter_func_from_csv neck neck-r neck-p + - 0)\n"
body="${body})\n"

echo -e "${body}" > "$(rospack find aero_ros_bridge)/euslisp/aero-upper-strokes.l"

body=''

body="${body}(defmethod AeroRobotLower-robot\n"
body="${body}${tab2}(:stroke-vector ()\n"
body="${body}${tab2}${tab2}(let (av result)\n"
body="${body}${tab2}${tab2}${tab2}(setq av (send self :angle-vector))\n"
body="${body}${tab2}${tab2}${tab2}(setq result (float-vector\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 0)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :crotch-p-table (round (elt av 1)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :knee-p-table (round (elt av 2)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 3)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :crotch-p-table (round (elt av 4)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :knee-p-table (round (elt av 5)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 6)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :crotch-p-table (round (elt av 7)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :knee-p-table (round (elt av 8)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 9)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :crotch-p-table (round (elt av 10)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :knee-p-table (round (elt av 11)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}0\n"
body="${body}${tab2}${tab2}${tab2}${tab2}))\n"
body="${body}${tab2}${tab2}${tab2}result))\n"
body="${body}$(create_table_func_from_csv crotch-p 0)\n"
body="${body}$(create_table_func_from_csv knee-p 0)\n"
body="${body})\n"

echo -e "${body}" > "$(rospack find aero_ros_bridge)/euslisp/aero-lower-strokes.l"