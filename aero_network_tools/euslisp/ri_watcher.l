#!/usr/bin/env roseus

(ros::roseus "ri_watcher")
(ros::load-ros-manifest "aero_network_tools")

(defun av_compressed_narrow_receiver_cb (msg)
  (let (av)
    (setq av (send msg :angleVector))
    (setq av (list (concatenate float-vector
				(subseq av 0 8)
				#f(0 0 0)
				(subseq av 8 16)
				#f(0 0 0)
				(subseq av 16 22))
		   (concatenate float-vector
				(subseq av 22 34)
				#f(0))))
    (setq av (list (float-vector (elt (elt av 0) 0) ;rsP
				 (elt (elt av 0) 1) ;rsR
				 (* (elt (elt av 0) 2) 2) ;rsY
				 (- (elt (elt av 0) 3) 50) ;re
				 (* (elt (elt av 0) 4) 2) ;rwY
				 (elt (elt av 0) 5) ;rwP
				 (elt (elt av 0) 6) ;rwR
				 (elt (elt av 0) 7) ;rt1
				 (elt (elt av 0) 8) ;rf1
				 (elt (elt av 0) 9) ;rf2
				 (elt (elt av 0) 10) ;rf3
				 (elt (elt av 0) 11) ;lsP
				 (elt (elt av 0) 12) ;lsR
				 (* (elt (elt av 0) 13) 2) ;lsY
				 (- (elt (elt av 0) 14) 50) ;le
				 (* (elt (elt av 0) 15) 2) ;lwY
				 (elt (elt av 0) 16) ;lwP
				 (elt (elt av 0) 17) ;lwR
				 (elt (elt av 0) 18) ;lt1
				 (elt (elt av 0) 19) ;lf1
				 (elt (elt av 0) 20) ;lf2
				 (elt (elt av 0) 21) ;lf3
				 (* (elt (elt av 0) 22) 2) ;wY
				 (elt (elt av 0) 23) ;wP
				 (elt (elt av 0) 24) ;wR
				 (* (elt (elt av 0) 25) 2) ;nY
				 (elt (elt av 0) 26) ;nP
				 (elt (elt av 0) 27)) ;nR
		   (float-vector (* (elt (elt av 1) 0) 2) ;rrY
				 (elt (elt av 1) 1) ;rrP
				 (elt (elt av 1) 2) ;rrkP
				 (* (elt (elt av 1) 3) 2) ;frY
				 (elt (elt av 1) 4) ;frP
				 (elt (elt av 1) 5) ;frkP
				 (* (elt (elt av 1) 6) 2) ;rlY
				 (elt (elt av 1) 7) ;rlP
				 (elt (elt av 1) 8) ;rlkP
				 (* (elt (elt av 1) 9) 2) ;flY
				 (elt (elt av 1) 10) ;flP
				 (elt (elt av 1) 11) ;flkP
				 (elt (elt av 1) 12)))) ;dummy
    (send *aero* :angle-vector av)
    (send *irtviewer* :draw-objects)
    ))

(ros::subscribe "/aero_network/ocs_av_narrow_band_compressed/receiver"
		aero_network_tools::AeroJointAnglesCompressed
		#'av_compressed_narrow_receiver_cb)

(load "package://aero_ros_bridge/euslisp/aero-interface.l")
(load "package://aero_ros_bridge/euslisp/walker.l")
(aero-init)
(send *aero* :draw-on)

(defun debug ()
  (do-until-key
   (ros::spin-once)))
