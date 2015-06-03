#!/usr/bin/env roseus

(ros::roseus "auto_aero_roseus")
(ros::load-ros-manifest "aero_network_tools")

(defvar *auto-aero-debug* nil)

(defclass auto-aero-roseus-class
  :super propertied-object
  :slots (last-msg msg-streak))
(defmethod auto-aero-roseus-class
  (:init ()
    (ros::advertise "/aero_network/resp_narrow_band/streamer" aero_network_tools::AeroNarrowResponse)
    (ros::advertise "/aero_network/fc_av_narrow_band/streamer" aero_network_tools::AeroJointAngles)
    (ros::advertise "/aero_network/av_narrow_band_compressed/streamer" aero_network_tools::AeroJointAnglesCompressed)
    (ros::subscribe
     "/aero_network/cmd_narrow_band/receiver"
     aero_network_tools::AeroNarrowCommand #'send self :cmd-cb)
    (ros::subscribe
     "/aero_network/fc_av_narrow_band/receiver"
     aero_network_tools::AeroJointAngles #'send self :av-cb)
    (setq last-msg -999)
    (setq msg-streak 0)
    )
  (:send-feedback ()
    (if (eq *auto-aero-debug* t)
	(setq tmp (send *aero* :angle-vector))
      (setq tmp (send *ri* :actual-vector)))
    (setq tmp (concatenate float-vector
			   (subseq (elt tmp 0) 0 8)
			   (subseq (elt tmp 0) 11 19)
			   (subseq (elt tmp 0) 22 28)
			   (subseq (elt tmp 1) 0 12)))
    (setq tmp (integer-vector (round (elt tmp 0)) ;rsP
			      (round (elt tmp 1)) ;rsR
			      (round (/ (elt tmp 2) 2)) ;rsY
			      (round (+ (elt tmp 3) 50)) ;re
			      (round (/ (elt tmp 4) 2)) ;rwY
			      (round (elt tmp 5)) ;rwP
			      (round (elt tmp 6)) ;rwR
			      (round (elt tmp 7)) ;rt1
			      (round (elt tmp 8)) ;lsP
			      (round (elt tmp 9)) ;lsR
			      (round (/ (elt tmp 10) 2)) ;lsY
			      (round (+ (elt tmp 11) 50)) ;le
			      (round (/ (elt tmp 12) 2)) ;lwY
			      (round (elt tmp 13)) ;lwP
			      (round (elt tmp 14)) ;lwR
			      (round (elt tmp 15)) ;lt1
			      (round (/ (elt tmp 16) 2)) ;wY
			      (round (elt tmp 17)) ;wP
			      (round (elt tmp 18)) ;wR
			      (round (/ (elt tmp 19) 2)) ;nY
			      (round (elt tmp 20)) ;nP
			      (round (elt tmp 21)) ;nR
			      (round (/ (elt tmp 22) 2)) ;rrY
			      (round (elt tmp 23)) ;rrP
			      (round (elt tmp 24)) ;rrkP
			      (round (/ (elt tmp 25) 2)) ;frY
			      (round (elt tmp 26)) ;frP
			      (round (elt tmp 27)) ;frkP
			      (round (/ (elt tmp 28) 2)) ;rlY
			      (round (elt tmp 29)) ;rlP
			      (round (elt tmp 30)) ;rlkP
			      (round (/ (elt tmp 31) 2)) ;flY
			      (round (elt tmp 32)) ;flP
			      (round (elt tmp 33)))) ;flkP
    (ros::publish "/aero_network/av_narrow_band_compressed/streamer"
		  (instance aero_network_tools::AeroJointAnglesCompressed :init
			    :angleVector tmp))
    )
  (:av-cb (msg)
    (setq tmp (send msg :angleVector))
    (send *aero* :angle-vector (list (concatenate float-vector
						  (subseq tmp 0 8)
						  #f(0.0 0.0 0.0)
						  (subseq tmp 8 16)
						  #f(0.0 0.0 0.0)
						  (subseq tmp 16 22))
				     (concatenate float-vector
						  (subseq tmp 22 34)
						  #f(0.0))))
    (send-angle-vector)
    (ros::publish "/aero_network/resp_narrow_band/streamer"
		  (instance aero_network_tools::AeroNarrowResponse :init
			    :cmd 127 :resp 0))
    (send self :send-feedback)
    )
  (:cmd-cb (msg)
    (let (r-msg av-msg (tmp))
      (setq r-msg (instance aero_network_tools::AeroNarrowResponse :init))
      (send r-msg :cmd (send msg :cmd))
      (if (= (send msg :cmd) last-msg)
	  (incf msg-streak) (setq msg-streak 0))
      (send r-msg :resp msg-streak)
      (case (send msg :cmd)
	    (-126 (setq av-msg
			(instance aero_network_tools::AeroJointAngles :init))
		  (if (eq *auto-aero-debug* t)
		      (setq tmp (send *aero* :angle-vector))
		    (setq tmp (send *ri* :actual-vector)))
		  (send av-msg :angleVector (concatenate float-vector
							 (subseq (elt tmp 0) 0 8)
							 (subseq (elt tmp 0) 11 19)
							 (subseq (elt tmp 0) 22 28)
							 (subseq (elt tmp 1) 0 12)))
		  (ros::publish "/aero_network/fc_av_narrow_band/streamer" av-msg)
		  )
	    (-7 (send *aero* :hand :arms :joint-angle (send msg :theta))
		(send-angle-vector))
	    (-5 (send *aero* :hand :arms :open-pose)
		(send-angle-vector))
	    (-3 (send *aero* :hand :arms :close-pose)
		(send-angle-vector))
	    (-1 (if (eq (send *aero* :upper :arms :move-end-pos
			      (float-vector (send msg :x)
					    (send msg :y)
					    (send msg :z))
			      (if (= (send msg :coords) 0)
				  :local :world))
			nil)
		    (send r-msg :resp -2)
		  (send-angle-vector)))
	    (0 (if (eq (send *aero* :upper :rarm :move-end-pos
			     (float-vector (send msg :x)
					   (send msg :y)
					   (send msg :z))
			     (if (= (send msg :coords) 0)
				 :local :world))
		       nil)
		   (send r-msg :resp -2)
		 (send-angle-vector)))
	    (1 (if (eq (send *aero* :upper :larm :move-end-pos
			     (float-vector (send msg :x)
					   (send msg :y)
					   (send msg :z))
			     (if (= (send msg :coords) 0)
				 :local :world))
		       nil)
		   (send r-msg :resp -2)
		 (send-angle-vector)))
	    (2 (send *aero* :hand :rarm :close-pose)
	       (send-angle-vector))
	    (3 (send *aero* :hand :larm :close-pose)
	       (send-angle-vector))
	    (4 (send *aero* :hand :rarm :open-pose)
	       (send-angle-vector))
	    (5 (send *aero* :hand :larm :open-pose)
	       (send-angle-vector))
	    (6 (send *aero* :hand :rarm :joint-angle (send msg :theta))
	       (send-angle-vector))
	    (7 (send *aero* :hand :larm :joint-angle (send msg :theta))
	       (send-angle-vector))
	    (8 (if (eq (send *aero* :move-waist
			     (float-vector (send msg :x)
					   (send msg :y)
					   (send msg :z)))
		       nil)
		   (send r-msg :resp -2)
		 (send-angle-vector)))
	    (10 (setq tmp (send *ri* :go-pos
				(send msg :x) (send msg :y) (send msg :theta)
				(send msg :coords)))
		(if (< tmp 0) (send r-msg :resp -3)))
	    (14 (send *aero* :upper :rarm :move-end-rot (deg2rad (send msg :theta))
		      (float-vector (send msg :x) (send msg :y) (send msg :z)))
		(send-angle-vector))
	    (15 (send *aero* :upper :larm :move-end-rot (deg2rad (send msg :theta))
		      (float-vector (send msg :x) (send msg :y) (send msg :z)))
		(send-angle-vector))
	    (16 (send *aero* (case (send msg :x)
				   (0 :reset-manip-pose)
				   (1 :reset-terrain-pose)
				   (t
				    (send r-msg :resp -4)
				    :angle-vector)
				   ))
		(send-angle-vector))
	    (17 (send *aero* :angle-vector
		      (send *ri* (case (send msg :x)
				       (0 :gp-thk-x)
				       (t
					(send r-msg :resp -4)
					:actual-vector)
				       )))
		(send-angle-vector))
	    (41 (rotate-knob :larm (send msg :y)))
	    (43 (open-door :larm (send msg :z)))
	    (49 (setup-door))
	    (50 (rotate-valve :rarm (send msg :x) (send msg :theta)))
	    (51 (rotate-valve :larm (send msg :x) (send msg :theta)))
	    (52 (release-arm :rarm))
	    (53 (release-arm :larm))
	    (59 (setup-valve))
	    (61 (if (eq (send *walker* :step
			      (float-vector (send msg :x) (send msg :y) (send msg :z))
			      :rarm :balance-rate-thre (send msg :theta)
			      :step-height (send msg :coords)) nil)
		    (send r-msg :resp -4)))
	    (62 (if (eq (send *walker* :step
			      (float-vector (send msg :x) (send msg :y) (send msg :z))
			      :larm :balance-rate-thre (send msg :theta)
			      :step-height (send msg :coords)) nil)
		    (send r-msg :resp -4)))
	    (63 (if (eq (send *walker* :step
			      (float-vector (send msg :x) (send msg :y) (send msg :z))
			      :lleg :balance-rate-thre (send msg :theta)
			      :step-height (send msg :coords)) nil)
		    (send r-msg :resp -4)))
	    (64 (if (eq (send *walker* :step
			      (float-vector (send msg :x) (send msg :y) (send msg :z))
			      :rleg :balance-rate-thre (send msg :theta)
			      :step-height (send msg :coords)) nil)
		    (send r-msg :resp -4)))
	    (65 (if (eq (send *walker* :step-square
			      (float-vector (send msg :x) (send msg :y) (send msg :z))
			      :rarm :balance-rate-thre (send msg :theta)
			      :step-height (send msg :coords)) nil)
		    (send r-msg :resp -4)))
	    (66 (if (eq (send *walker* :step-square
			      (float-vector (send msg :x) (send msg :y) (send msg :z))
			      :larm :balance-rate-thre (send msg :theta)
			      :step-height (send msg :coords)) nil)
		    (send r-msg :resp -4)))
	    (67 (if (eq (send *walker* :step-square
			      (float-vector (send msg :x) (send msg :y) (send msg :z))
			      :lleg :balance-rate-thre (send msg :theta)
			      :step-height (send msg :coords)) nil)
		    (send r-msg :resp -4)))
	    (68 (if (eq (send *walker* :step-square
			      (float-vector (send msg :x) (send msg :y) (send msg :z))
			      :rleg :balance-rate-thre (send msg :theta)
			      :step-height (send msg :coords)) nil)
		    (send r-msg :resp -4)))
	    (69 (if (eq (send *walker* :walk
			      :goal (float-vector (send msg :x) (send msg :y) 0)
			      :balance-rate-thre (send msg :theta)
			      :step-height (send msg :z)
			      :stride-length (send msg :coords)) nil)
		    (send r-msg :resp -4)))
	    (100 (send *aero* :upper :reach-arm :rarm :stop (send msg :z)
		       :toward (case (send msg :x)
				     (1 #f(1 0 0))
				     (2 #f(0 1 0))
				     (3 #f(0 0 1))
				     (-1 #f(-1 0 0))
				     (-2 #f(0 -1 0))
				     (-3 #f(0 0 -1))
				     (t (send r-msg :resp -4) #f(0 0 0)))
		       :adjust (case (send msg :y)
				     (1 #f(1 0 0))
				     (2 #f(0 1 0))
				     (3 #f(0 0 1))
				     (-1 #f(-1 0 0))
				     (-2 #f(0 -1 0))
				     (-3 #f(0 0 -1))
				     (t (send r-msg :resp -4) #f(0 0 0))))
		 (send-angle-vector))
	    (101 (send *aero* :upper :reach-arm :larm :stop (send msg :z)
		       :toward (case (send msg :x)
				     (1 #f(1 0 0))
				     (2 #f(0 1 0))
				     (3 #f(0 0 1))
				     (-1 #f(-1 0 0))
				     (-2 #f(0 -1 0))
				     (-3 #f(0 0 -1))
				     (t (send r-msg :resp -4) #f(0 0 0)))
		       :adjust (case (send msg :y)
				     (1 #f(1 0 0))
				     (2 #f(0 1 0))
				     (3 #f(0 0 1))
				     (-1 #f(-1 0 0))
				     (-2 #f(0 -1 0))
				     (-3 #f(0 0 -1))
				     (t (send r-msg :resp -4) #f(0 0 0))))
		 (send-angle-vector))
	    (102 (send *aero* :magic-close :rarm))
	    (103 (send *aero* :magic-close :larm))
	    (104 (send *aero* :magic-open :rarm))
	    (105 (send *aero* :magic-open :larm))
	    (106 (send *aero* :hand-yaw-world :rarm (send msg :theta)))
	    (107 (send *aero* :hand-yaw-world :larm (send msg :theta)))
	    (t (send r-msg :resp -1))
	    )
      (setq last-msg (send msg :cmd))
      (ros::publish "/aero_network/resp_narrow_band/streamer" r-msg)
      (send self :send-feedback)
      ))
  )

(setq auto-aero-roseus (instance auto-aero-roseus-class :init))

(load "package://aero_ros_bridge/euslisp/aero-interface.l")
(load "package://aero_2015_final_drc/euslisp/aero-drc-testbed.l")
(load "package://aero_2015_final_drc/euslisp/include-aero-drc-door-task.l")
(load "package://aero_2015_final_drc/euslisp/include-aero-drc-valve-task.l")
(load "package://aero_ros_bridge/euslisp/walker.l")
;; (aero-init)
(make-drc-testbed-models-with-aero)
(setq *real* t)

(while (ros::ok)
  (ros::spin-once)
  )
