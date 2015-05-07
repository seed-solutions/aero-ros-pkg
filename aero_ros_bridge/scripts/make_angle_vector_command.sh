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
	if [[ $idx -lt $((${#table[@]} - 1)) ]]
	then
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(${idx} (setq candidates (list${e})) (setq appendix (list${table[$(($idx + 1))]})))\n"
	else
	    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(${idx} (setq candidates (list${e})))\n"
	fi
	idx=$(($idx + 1))
    done

    tab6='      '
    tab8='        '
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2}(t (return-from :rev-${1}-table nil))\n"
    lisp="${lisp}${tab2}${tab2}${tab2}${tab2})\n"
    lisp="${lisp}${tab6}(cond ((< stroke 0)\n"
    lisp="${lisp}${tab6}${tab8}(if (> (length candidates) 2)\n"
    lisp="${lisp}${tab6}${tab8}${tab2}(if (< (cadr (car candidates)) (cadr (cadr candidates))) (setq candidates (reverse candidates))))\n"
    lisp="${lisp}${tab6}${tab8}(dolist (x candidates)\n"
    lisp="${lisp}${tab6}${tab8}${tab2}(if (>= stroke (cadr x)) (progn (setq result (- (car x) (* (- (cadr x) stroke) (caddr x)))) (return))))\n"
    lisp="${lisp}${tab6}${tab8}(if (eq result nil)\n"
    lisp="${lisp}${tab6}${tab8}${tab2}(progn (if (> (length appendix) 2)\n"
    lisp="${lisp}${tab6}${tab8}${tab2}${tab8}(if (< (cadr (car appendix)) (cadr (cadr appendix))) (setq appendix (reverse appendix))))\n"
    lisp="${lisp}${tab6}${tab8}${tab2}${tab8}(setq result (- (car (car appendix)) (* (- (cadr (car appendix)) stroke) (caddr (car appendix)))))))\n"
    lisp="${lisp}${tab6}${tab8})\n"
    lisp="${lisp}${tab6}${tab6}((>= stroke 0)\n"
    lisp="${lisp}${tab6}${tab8}(if (> (length candidates) 2)\n"
    lisp="${lisp}${tab6}${tab8}${tab2}(if (> (cadr (car candidates)) (cadr (cadr candidates))) (setq candidates (reverse candidates))))\n"
    lisp="${lisp}${tab6}${tab8}(dolist (x candidates)\n"
    lisp="${lisp}${tab6}${tab8}${tab2}(if (<= stroke (cadr x)) (progn (setq result (- (car x) (* (- (cadr x) stroke) (caddr x)))) (return))))\n"
    lisp="${lisp}${tab6}${tab8}(if (eq result nil)\n"
    lisp="${lisp}${tab6}${tab8}${tab2}(progn (if (> (length appendix) 2)\n"
    lisp="${lisp}${tab6}${tab8}${tab2}${tab8}(if (> (cadr (car appendix)) (cadr (cadr appendix))) (setq appendix (reverse appendix))))\n"
    lisp="${lisp}${tab6}${tab8}${tab2}${tab8}(setq result (- (car (car appendix)) (* (- (cadr (car appendix)) stroke) (caddr (car appendix)))))))\n"
    lisp="${lisp}${tab6}))\n"
    lisp="${lisp}${tab2}${tab2}result)\n"

    echo -e "${lisp}"
}

body=''
tab2=$'  '

body="${body}(defmethod aero-upper-interface\n"
#body="${body}${tab2}(:stroke-to-angle ()\n"
body="${body}$(create_table_func_from_csv shoulder-p 0)\n"
body="${body})\n"

echo -e "${body}" > "$(rospack find aero_ros_bridge)/euslisp/aero-upper-angles.l"