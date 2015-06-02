(load "package://aero_2015_final_drc/euslisp/aero-drc-testbed.l")
(load "package://aero_2015_final_drc/euslisp/include-aero-drc-door-task.l")
(load "package://aero_2015_final_drc/euslisp/include-aero-drc-valve-task.l")
(load "package://aero_ros_bridge/euslisp/walker.l")

(ros::roseus "aero-drc-c-mode")
(ros::load-ros-manifest "aero_network_tools")

(ros::advertise "/aero_network/cmd_narrow_band/streamer" aero_network_tools::AeroNarrowCommand 50)
(ros::advertise "/aero_network/sv_narrow_band/streamer" aero_network_tools::AeroJointStrokes 50)

(defvar *task* nil)

(defun av_narrow_receiver_cb (msg)
  (let (av)
  (setq av (send msg :angleVector))
  (send *aero* :angle-vector
	(list (concatenate float-vector
			   (subseq av 0 8)
			   #f(0 0 0)
			   (subseq av 8 16)
			   #f(0 0 0)
			   (subseq av 16 22))
	      (concatenate float-vector
			   (subseq av 22 34)
			   #f(0)))))
  (if (boundp '*irtviewer*)
      (send *irtviewer* :draw-objects))
  )

(ros::subscribe "/aero_network/av_narrow_band/receiver"
		aero_network_tools::AeroJointAngles #'av_narrow_receiver_cb)

(defun ctrl- (cmd)
  (let (~vector~ ~arm~ ~coords~ ~pose~ (~cmd~ 127) (~x~ 0) (~y~ 0) (~z~ 0) (~theta~ 0) (~flag~ 0)
		 ~link~ ~joint~ ~angle~ ~balance-thre~ ~step-height~ ~stride-length~)
    (setq ~coords~ :local)
    (if (position `-w cmd) (progn (setq ~coords~ :world) (setq ~flag~ 1)))
    (if (position `-l cmd) (progn (setq ~coords~ :local) (setq ~flag~ 0)))
    (if (position `-t cmd) (progn (setq ~coords~ t) (setq ~flag~ 1)))
    (case (car cmd)
	  ;; common
	  (move-end-pos
	   (setq ~cmd~ 1)
	   (case (elt cmd 1)
		 (`l (setq ~arm~ :larm))
		 (`r (setq ~arm~ :rarm) (setq ~cmd~ (- ~cmd~ 1)))
		 (`lr (setq ~arm~ :arms) (setq ~cmd~ (* -1 ~cmd~)))
		 (t (return-from ctrl- (warn "no such arm~%"))))
	   (setq ~x~ (elt cmd 2))
	   (setq ~y~ (elt cmd 3))
	   (setq ~z~ (elt cmd 4))
	   (setq ~vector~ (float-vector ~x~ ~y~ ~z~))
	   (if (not (eq (send *aero* :upper ~arm~ :move-end-pos ~vector~ ~coords~) nil))
	       (ros::publish "/aero_network/cmd_narrow_band/streamer"
			     (instance aero_network_tools::AeroNarrowCommand :init
				       :cmd ~cmd~
				       :x ~x~
				       :y ~y~
				       :z ~z~
				       :coords ~flag~)))
	   )
	  (move-end-rot
	   (setq ~cmd~ 15)
	   (case (elt cmd 1)
		 (`l (setq ~arm~ :larm))
		 (`r (setq ~arm~ :rarm) (setq ~cmd~ (- ~cmd~ 1)))
		 (t (return-from ctrl- (warn "no such arm~%"))))
	   (setq ~theta~ (elt cmd 2))
	   (setq ~x~ (elt cmd 3))
	   (setq ~y~ (elt cmd 4))
	   (setq ~z~ (elt cmd 5))
	   (setq ~vector~ (float-vector ~x~ ~y~ ~z~))
	   (if (not (eq (send *aero* :upper ~arm~ :move-end-rot ~theta~ ~vector~ ~coords~) nil))
	       (ros::publish "/aero_network/cmd_narrow_band/streamer"
			     (instance aero_network_tools::AeroNarrowCommand :init
				       :cmd ~cmd~
				       :x ~x~
				       :y ~y~
				       :z ~z~
				       :theta ~theta~
				       :coords ~flag~)))
	   )
	  (reach-arm
	   (setq ~cmd~ 101)
	   (case (elt cmd 1)
		 (`l (setq ~arm~ :larm))
		 (`r (setq ~arm~ :rarm) (setq ~cmd~ (- ~cmd~ 1)))
		 (t (return-from ctrl- (warn "no such arm~%"))))
	   (setq ~vector~ (cond ((eq (elt cmd 2) `x) (setq ~x~ 1) #f(1 0 0))
				((eq (elt cmd 2) `-x) (setq ~x~ -1) #f(-1 0 0))
				((eq (elt cmd 2) `y)  (setq ~x~ 2) #f(0 1 0))
				((eq (elt cmd 2) `-y) (setq ~x~ -2) #f(0 -1 0))
				((eq (elt cmd 2) `z) (setq ~x~ 3) #f(0 0 1))
				((eq (elt cmd 2) `-z) (setq ~x~ -3) #f(0 0 -1))))
	   (setq ~vector2~ (cond ((eq (elt cmd 2) `x) (setq ~y~ 1) #f(1 0 0))
				((eq (elt cmd 2) `-x) (setq ~y~ -1) #f(-1 0 0))
				((eq (elt cmd 2) `y)  (setq ~y~ 2) #f(0 1 0))
				((eq (elt cmd 2) `-y) (setq ~y~ -2) #f(0 -1 0))
				((eq (elt cmd 2) `z) (setq ~y~ 3) #f(0 0 1))
				((eq (elt cmd 2) `-z) (setq ~y~ -3) #f(0 0 -1))))
	   (setq ~time~ (elt cmd 4))
	   (send *aero* :upper :reach-arm ~arm~ :toward ~vector~ :adjust ~vector2~ :stop ~time~)
	   (ros::publish "/aero_network/cmd_narrow_band/streamer"
			 (instance aero_network_tools::AeroNarrowCommand :init
				   :cmd ~cmd~
				   :x ~x~
				   :y ~y~
				   :z ~time~))
	   )
	  (hand
	   (setq ~pose~ (elt cmd 2))
	   (case ~pose~
		 (:close-pose (setq ~cmd~ 3))
		 (:open-pose (setq ~cmd~ 5))
		 (:joint-angle (setq ~cmd~ 7) (setq ~theta~ (elt cmd 3)))
		 (t (return-from ctrl- (warn "no such pose~%"))))
	   (case (elt cmd 1)
		 (`l (setq ~arm~ :larm))
		 (`r (setq ~arm~ :rarm) (setq ~cmd~ (- ~cmd~ 1)))
		 (`lr (setq ~arm~ :arms) (setq ~cmd~ (* -1 ~cmd~)))
		 (t (return-from ctrl- (warn "no such arm~%"))))
	   (send *aero* :hand ~arm~ ~pose~)
	   (ros::publish "/aero_network/cmd_narrow_band/streamer"
			 (instance aero_network_tools::AeroNarrowCommand :init
				   :cmd ~cmd~
				   :theta ~theta~))
	   )
	  (magic-close
	   (setq ~cmd~ 103)
	   (case (elt cmd 1)
		 (`l (setq ~arm~ :larm))
		 (`r (setq ~arm~ :rarm) (setq ~cmd~ (- ~cmd~ 1)))
		 (t (return-from ctrl- (warn "no such arm~%"))))
	   (send *aero* :magic-close ~arm~)
	   (ros::publish "/aero_network/cmd_narrow_band/streamer"
			 (instance aero_network_tools::AeroNarrowCommand :init
				   :cmd ~cmd~))
	   )
	  (magic-open
	   (setq ~cmd~ 105)
	   (case (elt cmd 1)
		 (`l (setq ~arm~ :larm))
		 (`r (setq ~arm~ :rarm) (setq ~cmd~ (- ~cmd~ 1)))
		 (t (return-from ctrl- (warn "no such arm~%"))))
	   (send *aero* :magic-open ~arm~)
	   (ros::publish "/aero_network/cmd_narrow_band/streamer"
			 (instance aero_network_tools::AeroNarrowCommand :init
				   :cmd ~cmd~))
	   )
	  (hand-yaw-world
	   (setq ~cmd~ 107)
	   (case (elt cmd 1)
		 (`l (setq ~arm~ :larm))
		 (`r (setq ~arm~ :rarm) (setq ~cmd~ (- ~cmd~ 1)))
		 (t (return-from ctrl- (warn "no such arm~%"))))
	   (setq ~theta~ (elt cmd 2))
	   (send *aero* :hand-yaw-world ~arm~ ~theta~)
	   (ros::publish "/aero_network/cmd_narrow_band/streamer"
			 (instance aero_network_tools::AeroNarrowCommand :init
				   :cmd ~cmd~
				   :theta ~theta~))
	   )
	  (move-waist
	   (setq ~cmd~ 8)
	   (setq ~x~ (elt cmd 1))
	   (setq ~y~ (elt cmd 2))
	   (setq ~z~ (elt cmd 3))
	   (setq ~vector~ (float-vector ~x~ ~y~ ~z~))
	   (if (not (eq (send *aero* :move-waist ~vector~) nil))
	       (ros::publish "/aero_network/cmd_narrow_band/streamer"
			     (instance aero_network_tools::AeroNarrowCommand :init
				       :cmd ~cmd~
				       :x ~x~
				       :y ~y~
				       :z ~z~)))
	   )
	  (go-pos
	   (setq ~cmd~ 10)
	   (setq ~x~ (elt cmd 1))
	   (setq ~y~ (elt cmd 2))
	   (setq ~z~ (elt cmd 3))
	   (if (>= (send *ri* :go-pos ~x~ ~y~ ~z~ (if (= ~flag~ 0) nil t)) 0)
	       (ros::publish "/aero_network/cmd_narrow_band/streamer"
			     (instance aero_network_tools::AeroNarrowCommand :init
				       :cmd ~cmd~
				       :x ~x~
				       :y ~y~
				       :z ~z~
				       :coords ~flag~))
	     (warn "detected warning! was not sent~%"))
	   )
	  (j
	   (setq ~link~ (elt cmd 1))
	   (setq ~joint~ (elt cmd 2))
	   (setq ~angle~ (elt cmd 3))
	   (send *aero* :upper ~link~ ~joint~ :joint-angle ~angle~)
	   )
	  (reset-manip-pose
	   (send *aero* :reset-manip-pose)
	   (ros::publish "/aero_network/cmd_narrow_band/streamer"
			 (instance aero_network_tools::AeroNarrowCommand :init
				   :cmd 16
				   :x 0))
	   )
	  (reset-terrain-pose
	   (send *aero* :reset-terrain-pose)
	   (ros::publish "/aero_network/cmd_narrow_band/streamer"
			 (instance aero_network_tools::AeroNarrowCommand :init
				   :cmd 16
				   :x 1))
	   )
	  (gp-thk-x
	   (send *aero* :angle-vector (send *ri* :gp-thk-x))
	   (ros::publish "/aero_network/cmd_narrow_band/streamer"
			 (instance aero_network_tools::AeroNarrowCommand :init
				   :cmd 17
				   :x 0))
	   )
	  (send-angle-vector
	   (send-angle-vector)
	   (setq ~vector~ (send *aero* :stroke-vector))
	   (ros::publish "/aero_network/sv_narrow_band/streamer"
			 (instance aero_network_tools::AeroJointStrokes :init
				   :strokeVector
				   (concatenate float-vector
						(subseq ~vector~ 0 8)
						(subseq ~vector~ 11 19)
						(subseq ~vector~ 22 41))))
	   )
	  (actual-vector
	   (ros::publish "/aero_network/cmd_narrow_band/streamer"
			 (instance aero_network_tools::AeroNarrowCommand :init
				   :cmd -126))
	   (read-line)
	   (ros::spin-once)
	   )
	  (rotate-knob
	   (if (not (equal *task* `door))
	       (return-from ctrl- (warn "not at door task")))
	   (setq ~cmd~ 41)
	   (case (elt cmd 1)
		 (`l (setq ~arm~ :larm))
		 (`r (setq ~arm~ :rarm) (setq ~cmd~ (- ~cmd~ 1)))
		 (t (return-from ctrl- (warn "no such arm~%"))))
	   (if (> (length cmd) 3) (setq ~y~ (elt cmd 2)) (setq ~y~ 0))
	   (rotate-knob ~arm~ ~y~)
	   (ros::publish "/aero_network/cmd_narrow_band/streamer"
			 (instance aero_network_tools::AeroNarrowCommand :init
				   :cmd ~cmd~
				   :y ~y~))
	   )
	  (open-door
	   (if (not (equal *task* `door))
	       (return-from ctrl- (warn "not at door task")))
	   (setq ~cmd~ 43)
	   (case (elt cmd 1)
		 (`l (setq ~arm~ :larm))
		 (`r (setq ~arm~ :rarm) (setq ~cmd~ (- ~cmd~ 1)))
		 (t (return-from ctrl- (warn "no such arm~%"))))
	   (if (> (length cmd) 3) (setq ~z~ (elt cmd 2)) (setq ~z~ 0))
	   (open-door ~arm~ ~z~)
	   (ros::publish "/aero_network/cmd_narrow_band/streamer"
			 (instance aero_network_tools::AeroNarrowCommand :init
				   :cmd ~cmd~
				   :z ~z~))
	   )
	  (setup-door
	   (setup-door)
	   (objects (list *door*))
	   (send *aero* :draw-on)
	   (setq *task* 'door)
	   (ros::publish "/aero_network/cmd_narrow_band/streamer"
			 (instance aero_network_tools::AeroNarrowCommand :init
				   :cmd 49))
	   )
	  (rotate-valve
	   (if (not (equal *task* `valve))
	       (return-from ctrl- (warn "not at valve task")))
	   (setq ~cmd~ 51)
	   (case (elt cmd 1)
		 (`l (setq ~arm~ :larm))
		 (`r (setq ~arm~ :rarm) (setq ~cmd~ (- ~cmd~ 1)))
		 (t (return-from ctrl- (warn "no such arm~%"))))
	   (setq ~x~ (elt cmd 2))
	   (setq ~theta~ (elt cmd 3))
	   (rotate-valve ~arm~ ~x~ ~theta~)
	   (ros::publish "/aero_network/cmd_narrow_band/streamer"
			 (instance aero_network_tools::AeroNarrowCommand :init
				   :cmd ~cmd~
				   :x ~x~
				   :theta ~theta~))
	   )
	  (release-arm
	   (if (not (equal *task* `valve))
	       (return-from ctrl- (warn "not at valve task")))
	   (setq ~cmd~ 53)
	   (case (elt cmd 1)
		 (`l (setq ~arm~ :larm))
		 (`r (setq ~arm~ :rarm) (setq ~cmd~ (- ~cmd~ 1)))
		 (t (return-from ctrl- (warn "no such arm~%"))))
	   (release-arm ~arm~)
	   (ros::publish "/aero_network/cmd_narrow_band/streamer"
			 (instance aero_network_tools::AeroNarrowCommand :init
				   :cmd ~cmd~))
	   )
	  (setup-valve
	   (setup-valve)
	   (objects (list *drc-valve*))
	   (send *aero* :draw-on)
	   (setq *task* 'valve)
	   (ros::publish "/aero_network/cmd_narrow_band/streamer"
			 (instance aero_network_tools::AeroNarrowCommand :init
				   :cmd 59))
	   )
	  (step
	   (setq ~cmd~ 64)
	   (case (elt cmd 1)
		 (`fr (setq ~arm~ :rarm) (setq ~cmd~ (- ~cmd~ 3)))
		 (`fl (setq ~arm~ :larm) (setq ~cmd~ (- ~cmd~ 2)))
		 (`rl (setq ~arm~ :lleg) (setq ~cmd~ (- ~cmd~ 1)))
		 (`rr (setq ~arm~ :rleg) (setq ~cmd~ (- ~cmd~ 3)))
		 (t (return-from ctrl- (warn "no such leg~%"))))
	   (setq ~x~ (elt cmd 2))
	   (setq ~y~ (elt cmd 3))
	   (setq ~z~ (elt cmd 4))
	   (setq ~vector~ (float-vector ~x~ ~y~ ~z~))
	   (if (position `-br cmd)
	       (setq ~balance-thre~ (elt cmd (+ (position `-br cmd) 1)))
	     (setq ~balance-thre~ 2.0))
	   (if (position `-sh cmd)
	       (setq ~step-height~ (elt cmd (+ (position `-sh cmd) 1)))
	     (setq ~step-height~ 100))
	   (if (eq (send *walker* :step ~vector~ ~arm~
			 :balance-rate-thre ~balance-thre~
			 :step-height ~step-height~) nil)
	       (ros::publish "/aero_network/cmd_narrow_band/streamer"
			     (instance aero_network_tools::AeroNarrowCommand :init
				       :cmd ~cmd~
				       :x ~x~
				       :y ~y~
				       :z ~z~
				       :theta ~balance-thre~
				       :coords ~step-height~)))
	     )
	  (step-square
	   (setq ~cmd~ 68)
	   (case (elt cmd 1)
		 (`fr (setq ~arm~ :rarm) (setq ~cmd~ (- ~cmd~ 3)))
		 (`fl (setq ~arm~ :larm) (setq ~cmd~ (- ~cmd~ 2)))
		 (`rl (setq ~arm~ :lleg) (setq ~cmd~ (- ~cmd~ 1)))
		 (`rr (setq ~arm~ :rleg) (setq ~cmd~ (- ~cmd~ 3)))
		 (t (return-from ctrl- (warn "no such leg~%"))))
	   (setq ~x~ (elt cmd 2))
	   (setq ~y~ (elt cmd 3))
	   (setq ~z~ (elt cmd 4))
	   (setq ~vector~ (float-vector ~x~ ~y~ ~z~))
	   (if (position `-br cmd)
	       (setq ~balance-thre~ (elt cmd (+ (position `-br cmd) 1)))
	     (setq ~balance-thre~ 2.0))
	   (if (position `-sh cmd)
	       (setq ~step-height~ (elt cmd (+ (position `-sh cmd) 1)))
	     (setq ~step-height~ 100))
	   (if (not (eq (send *walker* :step-square ~vector~ ~arm~
			      :balance-rate-thre ~balance-thre~
			      :step-height ~step-height~) nil))
	       (ros::publish "/aero_network/cmd_narrow_band/streamer"
			     (instance aero_network_tools::AeroNarrowCommand :init
				       :cmd ~cmd~
				       :x ~x~
				       :y ~y~
				       :z ~z~
				       :theta ~balance-thre~
				       :coords ~step-height~)))
	     )
	  (walk
	   (setq ~x~ (elt cmd 2))
	   (setq ~y~ (elt cmd 3))
	   (setq ~vector~ (float-vector ~x~ ~y~ 0))
	   (if (position `-br cmd)
	       (setq ~balance-thre~ (elt cmd (+ (position `-br cmd) 1)))
	     (setq ~balance-thre~ 2.0))
	   (if (position `-sh cmd)
	       (setq ~z~ (elt cmd (+ (position `-sh cmd) 1)))
	     (setq ~z~ 100))
	   (if (position '-sl cmd)
	       (setq ~stride-length~ (elt cmd (+ (position `-sl cmd) 1)))
	     (setq ~stride-length~ 120))
	   (if (not (eq (send *walker* :walk :goal ~vector~
			      :balance-rate-thre ~balance-thre~
			      :step-height ~z~
			      :stride-length ~stride-length~) nil))
	       (ros::publish "/aero_network/cmd_narrow_band/streamer"
			     (instance aero_network_tools::AeroNarrowCommand :init
				       :cmd ~cmd~
				       :x ~x~
				       :y ~y~
				       :z ~z~
				       :theta ~balance-thre~
				       :coords ~stride-length~)))
	   )
	  (t (warn "invalid command~%")))
    ))

(defun c-mode ()
  (dotimes (i 10000)
    (setq ~ui~ (read-line))
    (if (equal ~ui~ "q")
	(return-from c-mode nil))
    (ctrl- (read-from-string ~ui~))
    (if (boundp '*irtviewer*)
	(send *irtviewer* :draw-objects))
    ))

(make-drc-testbed-models-with-aero)
