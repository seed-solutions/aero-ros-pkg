#!/bin/bash

# rsP,rsR,rsY,re,rwY,rwP,rwR,rt1,rf1,rf2,rf3,lsP,lsR,lsY,le,lwY,lwP,lwR,lt1,lf1,lf2,lf3,wY,wP,wR,nY,nP,nR
#                   ,rwT,rwB,   ,nil,nil,nil,                  ,lwT,lwB,   ,nil,nil,nil,  ,wR,wL,  ,nR,nL

create_table_func_from_csv() { # joint_name table/real_origin_diff
    # load srv
    table=()
    file="$(rospack find aero_ros_bridge)/models/${1}.csv"
    j=0
    while read line
    do
	table[j]=$(echo "$line" | cut -d ',' -f4) 
	j=$(($j + 1))
    done < $file

    # create lisp method for angle-stroke-table
    tab2=$'  '
    strokes=''
    lisp="${tab2}(:${1}-table (&optional (angle nil))\n"
    lisp="${lisp}${tab2}${tab2}(case angle\n"

    idx=0
    for e in "${table[@]}"
    do
	angle=$(($2 + $idx))
	lisp="${lisp}${tab2}${tab2}${tab2}(${angle} ${e})\n"
	strokes="${strokes} ${e}"
	idx=$(($idx + 1))
    done
    
    lisp="${lisp}${tab2}${tab2}${tab2}(t (float-vector${strokes}))\n"
    lisp="${lisp}${tab2}${tab2}${tab2})\n"
    lisp="${lisp}${tab2}${tab2})\n"

    echo -e "${lisp}"
}

create_rp_converter_func_from_csv() { # joint_name roll-file pitch-file +/- +/-
    # load roll srv
    table_r=()
    file="$(rospack find aero_ros_bridge)/models/${2}.csv"
    j=0
    while read line
    do
	table_r[j]=$(echo "$line" | cut -d ',' -f4) 
	j=$(($j + 1))
    done < $file
    table_p=()
    # load pitch csv
    file="$(rospack find aero_ros_bridge)/models/${3}.csv"
    j=0
    while read line
    do
	table_p[j]=$(echo "$line" | cut -d ',' -f4) 
	j=$(($j + 1))
    done < $file

    # create lisp method for angle-stroke-table
    tab2=$'  '
    lisp="${tab2}(:${1}-rp-table (&optional (angle-r nil) (angle-p nil))\n"
    lisp="${lisp}${tab2}${tab2}(let ((roll-stroke nil) (pitch-stroke nil))\n"

    strokes=''
    idx=0
    lisp="${lisp}${tab2}${tab2}${tab2}(setq roll-stroke\n"
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(case angle-r\n"
    for e in "${table_r[@]}" # assumes model and real origin is 0
    do
	if [[ ${idx} -ne 0 ]]
	then
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab2}(-${idx} -${e})\n"
	fi
	lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab2}(${idx} ${e})\n"
	strokes="${strokes} ${e}"
	idx=$(($idx + 1))
    done
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab2}(t (float-vector${strokes}))\n"
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab2}))\n"
    strokes=''
    idx=0
    lisp="${lisp}${tab2}${tab2}${tab2}(setq pitch-stroke\n"
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(case angle-p\n"    
    for e in "${table_p[@]}" # assumes model and real origin is 0
    do
	lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab2}(${idx} ${e})\n"
	strokes="${strokes} ${e}"
	idx=$(($idx + 1))
    done
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab2}(t (float-vector${strokes}))\n"
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}${tab2}))\n"

    # calculate result in lisp
    lisp="${lisp}${tab2}${tab2}${tab2}(if (or (not pitch-stroke) (not roll-stroke))\n"
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}nil (float-vector (${4} pitch-stroke roll-stroke) (${5} pitch-stroke roll-stroke))))\n"
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
body="${body}${tab2}${tab2}${tab2}(setq right-wrist-tb (send self :wrist-rp-table (- (round (elt av 5))) (round (elt av 6))))\n"
body="${body}${tab2}${tab2}${tab2}(setq left-wrist-tb (send self :wrist-rp-table (- (round (elt av 16))) (round (elt av 17))))\n"
body="${body}${tab2}${tab2}${tab2}(setq waist-rl (send self :waist-rp-table (- (round (elt av 24))) (round (elt av 23))))\n"
body="${body}${tab2}${tab2}${tab2}(setq neck-rl (send self :neck-rp-table (- (round (elt av 27))) (round (elt av 26))))\n"
body="${body}${tab2}${tab2}${tab2}(setq result (float-vector\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :shoulder-p-table (- (round (elt av 0))))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :shoulder-r-table (- (round (elt av 1))))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 2)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :elbow-p-table (- (round (elt av 3))))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 4)\n"
#body="${body}${tab2}${tab2}${tab2}${tab2}(send self :wrist-p-table (- (round (elt av 5))))\n"
#body="${body}${tab2}${tab2}${tab2}${tab2}(send self :wrist-r-table (round (elt av 6)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt right-wrist-tb 0)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt right-wrist-tb 1)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 7)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 8)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 9)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 10)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :shoulder-p-table (- (round (elt av 11))))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :shoulder-r-table (round (elt av 12)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(elt av 13)\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :elbow-p-table (- (round (elt av 14))))\n"
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
body="${body}$(create_table_func_from_csv shoulder-p 0 1)\n"
body="${body}$(create_table_func_from_csv shoulder-r 0 1)\n"
body="${body}$(create_table_func_from_csv elbow-p 0 1)\n"
#body="${body}$(create_table_func_from_csv wrist-r 0 1)\n"
#body="${body}$(create_table_func_from_csv wrist-p 0 1)\n"
#body="${body}$(create_table_func_from_csv waist-p 0 1)\n"
#body="${body}$(create_table_func_from_csv waist-r 0 1)\n"
#body="${body}$(create_table_func_from_csv neck-p 0 1)\n"
#body="${body}$(create_table_func_from_csv neck-r 0 1)\n"
body="${body}$(create_rp_converter_func_from_csv wrist wrist-p wrist-r + -)\n"
body="${body}$(create_rp_converter_func_from_csv waist waist-r waist-p - +)\n"
body="${body}$(create_rp_converter_func_from_csv neck neck-r neck-p + -)\n"
body="${body})\n"

echo -e "${body}" > "$(rospack find aero_ros_bridge)/euslisp/aero-upper-strokes.l"

body=''

body="${body}(defmethod AeroRobotLower-robot\n"
body="${body}${tab2}(:stroke-vector ()\n"
body="${body}${tab2}${tab2}(let (av result)\n"
body="${body}${tab2}${tab2}${tab2}(setq av (send self :angle-vector))\n"
body="${body}${tab2}${tab2}${tab2}(setq result (float-vector\n"
body="${body}${tab2}${tab2}${tab2}${tab2}0\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :crotch-p-table (round (elt av 1)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :knee-p-table (round (elt av 2)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}0\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :crotch-p-table (round (elt av 4)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :knee-p-table (round (elt av 5)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}0\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :crotch-p-table (round (elt av 7)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :knee-p-table (round (elt av 8)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}0\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :crotch-p-table (round (elt av 10)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :knee-p-table (round (elt av 11)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}0\n"
body="${body}${tab2}${tab2}${tab2}${tab2}))\n"
body="${body}${tab2}${tab2}${tab2}result))\n"
body="${body}$(create_table_func_from_csv crotch-p 0 1)\n"
body="${body}$(create_table_func_from_csv knee-p 0 1)\n"
body="${body})\n"

echo -e "${body}" > "$(rospack find aero_ros_bridge)/euslisp/aero-lower-strokes.l"