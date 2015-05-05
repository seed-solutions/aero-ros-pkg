#!/bin/bash

create_table_func_from_csv() { # joint_name min/max_angle_in_eus ascending/descending
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
	idx=$(($idx + $3))
    done
    
    lisp="${lisp}${tab2}${tab2}${tab2}(t (float-vector${strokes}))\n"
    lisp="${lisp}${tab2}${tab2}${tab2})\n"
    lisp="${lisp}${tab2}${tab2})\n"

    echo -e "${lisp}"
}

body=''
tab2=$'  '

body="${body}(defmethod AeroUpperRobot-robot\n"
body="${body}${tab2}(:stroke-vector ()\n"
body="${body}${tab2}${tab2}(let (av result)\n"
body="${body}${tab2}${tab2}${tab2}(setq av (send self :angle-vector))\n"
body="${body}${tab2}${tab2}${tab2}(setq result (float-vector\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :shoulder-p-table (round (elt av 0)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :shoulder-r-table (round (elt av 1)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}0\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :elbow-p-table (round (elt av 3)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}0\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :wrist-p-table (round (elt av 5)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :wrist-r-table (round (elt av 6)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}0\n"
body="${body}${tab2}${tab2}${tab2}${tab2}0\n"
body="${body}${tab2}${tab2}${tab2}${tab2}0\n"
body="${body}${tab2}${tab2}${tab2}${tab2}0\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :shoulder-p-table (round (elt av 11)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :shoulder-r-table (round (elt av 12)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}0\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :elbow-p-table (round (elt av 14)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}0\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :wrist-p-table (round (elt av 16)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :wrist-r-table (round (elt av 17)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}0\n"
body="${body}${tab2}${tab2}${tab2}${tab2}0\n"
body="${body}${tab2}${tab2}${tab2}${tab2}0\n"
body="${body}${tab2}${tab2}${tab2}${tab2}0\n"
body="${body}${tab2}${tab2}${tab2}${tab2}0\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :waist-p-table (round (elt av 23)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :waist-r-table (round (elt av 24)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}0\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :neck-p-table (round (elt av 26)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}(send self :neck-r-table (round (elt av 27)))\n"
body="${body}${tab2}${tab2}${tab2}${tab2}))\n"
body="${body}${tab2}${tab2}${tab2}result))\n"
body="${body}$(create_table_func_from_csv shoulder-p 0 1)\n"
body="${body}$(create_table_func_from_csv shoulder-r 0 1)\n"
body="${body}$(create_table_func_from_csv elbow-p 0 1)\n"
body="${body}$(create_table_func_from_csv wrist-r 0 1)\n"
body="${body}$(create_table_func_from_csv wrist-p 0 1)\n"
body="${body}$(create_table_func_from_csv waist-p 0 1)\n"
body="${body}$(create_table_func_from_csv waist-r 0 1)\n"
body="${body}$(create_table_func_from_csv neck-p 0 1)\n"
body="${body}$(create_table_func_from_csv neck-r 0 1)\n"
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