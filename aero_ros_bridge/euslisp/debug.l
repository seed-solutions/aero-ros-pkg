(load "package://aero_ros_bridge/euslisp/aero-interface.l")

;; this file tests aero's methods

(defmethod aero-upper-interface
  (:debug-stroke (stroke-vector)
     (let ((joint-names (list "r_shoulder_pitch_joint"
			      "r_shoulder_roll_joint"
			      "r_elbow_yaw_joint"
			      "r_elbow_pitch_joint"
			      "r_wrist_roll_joint"
			      "r_wrist_top_joint"
			      "r_wrist_bottom_joint"
			      "r_hand_joint"
			      "dummy1"
			      "dummy2"
			      "dummy3"
			      "l_shoulder_pitch_joint"
			      "l_shoulder_roll_joint"
			      "l_elbow_yaw_joint"
			      "l_elbow_pitch_joint"
			      "l_wrist_roll_joint"
			      "l_wrist_top_joint"
			      "l_wrist_bottom_joint"
			      "l_hand_joint"
			      "dummy4"
			      "dummy5"
			      "dummy6"
			      "waist_pitch_joint"
			      "waist_right_joint"
			      "waist_left_joint"
			      "neck_yaw_joint"
			      "neck_right_joint"
			      "neck_left_joint"
			      "r_r_crotch_yaw_joint"
			      "r_r_crotch_pitch_joint"
			      "r_r_knee_pitch_joint"
			      "f_r_crotch_yaw_joint"
			      "f_r_crotch_pitch_joint"
			      "f_r_knee_pitch_joint"
			      "r_l_crotch_yaw_joint"
			      "r_l_crotch_pitch_joint"
			      "r_l_knee_pitch_joint"
			      "f_l_crotch_yaw_joint"
			      "f_l_crotch_pitch_joint"
			      "f_l_knee_pitch_joint"
			      "dummy7"))
	   (i 0)
	   (result nil))
       (dolist (x joint-names)
       	 (setq result (cons (cons x  (elt stroke-vector i)) result))
       	 (setq i (+ i 1)))
       result))
)

(aero-init)

(send *aero* :reset-manip-pose)
(send *aero* :draw-on)

(defun debug-reset ()
  (send *aero* :reset-manip-pose)
  )

(defun stroke-vector-test ()
  (send *aero* :reset-manip-pose)
  (send *aero* :stroke-vector)
  )

(defun go? (cmd &optional (real nil))
  (eval cmd)
  (print cmd)
  (send *irtviewer* :draw-objects)
  (if (eq real t)
      (progn
	(print "send real?")
	(do-until-key)
	(send *ri* :angle-vector (send *aero* :stroke-vector)))
    (do-until-key))
  )


(defun av-v-test ()
  (format nil "~A ~A" (send *aero* :angle-vector)
	  (send *ri* :stroke-to-angle
		(send *ri* :debug-stroke (send *aero* :stroke-vector))))
  )

(defun angle-vector-test (&key (virtual nil))
  (if (eq virtual t)    
      (send *aero* :angle-vector
	    (send *ri* :stroke-to-angle
		  (send *ri* :debug-stroke (send *aero* :stroke-vector))))
    (send *aero* :angle-vector (send *ri* :actual-vector)))
  )

(defun stair-test (&key (real nil))
  (go? `(send *aero* :reset-terrain-pose) real)
  (go? `(send *aero* :move-waist #f(0 0 -100)) real)
  (go? `(send *aero* :lower :rleg :move-end-pos #f(-100 -100 0) :world :rotation-axis nil) real)
  (go? `(send *aero* :move-waist #f(-100 -100 0)) real)
  (go? `(send *aero* :lower :larm :move-end-pos #f(0 0 100) :world :rotation-axis nil) real)
  (go? `(send *aero* :lower :larm :move-end-pos #f(0 0 100) :world :rotation-axis nil) real)
  (go? `(send *aero* :lower :larm :move-end-pos #f(0 0 -20) :world :rotation-axis nil) real)
  (go? `(send *aero* :move-waist #f(0 0 100)) real)
  (go? `(send *aero* :lower :lleg :move-end-pos #f(-200 0 0) :world :rotation-axis nil) real)
  (go? `(send *aero* :move-waist #f(-50 50 0)) real)
  (go? `(send *aero* :lower :rarm :move-end-pos #f(0 0 100) :world :rotation-axis nil) real)
  (go? `(send *aero* :lower :rarm :move-end-pos #f(0 0 100) :world :rotation-axis nil) real)
  (go? `(send *aero* :lower :rarm :move-end-pos #f(0 0 -20) :world :rotation-axis nil) real)
  (go? `(send *aero* :lower :rarm :move-end-pos #f(0 0 180) :world :rotation-axis nil) real)
  (go? `(send *aero* :lower :rarm :move-end-pos #f(150 0 0) :world :rotation-axis nil) real)
  (go? `(send *aero* :lower :rarm :move-end-pos #f(0 0 -50) :world :rotation-axis nil) real)
  (go? `(send *aero* :lower :larm :move-end-pos #f(0 0 10) :world :rotation-axis nil) real)
  (go? `(send *aero* :lower :larm :move-end-pos #f(0 0 50) :world :rotation-axis nil) real)
  (go? `(send *aero* :lower :larm :move-end-pos #f(0 0 50) :world :rotation-axis nil) real)
  (go? `(send *aero* :lower :larm :move-end-pos #f(0 0 50) :world :rotation-axis nil) real)
  (go? `(send *aero* :lower :larm :move-end-pos #f(0 0 40) :world :rotation-axis nil) real)
  (go? `(send *aero* :lower :larm :move-end-pos #f(150 -50 0) :world :rotation-axis nil) real)
  (go? `(send *aero* :lower :larm :move-end-pos #f(0 0 -60) :world :rotation-axis nil) real)
  (go? `(send *aero* :lower :lleg :move-end-pos #f(100 0 0) :world :rotation-axis nil) real)
  (go? `(send *aero* :move-waist #f(100 0 0)) real)
  (go? `(send *aero* :lower :lleg :move-end-pos #f(100 0 0) :world :rotation-axis nil) real)
  (go? `(send *aero* :lower :rleg :move-end-pos #f(100 0 0) :world :rotation-axis nil) real)
  (go? `(send *aero* :move-waist #f(100 0 0)) real)
  (go? `(send *aero* :lower :lleg :move-end-pos #f(100 0 0) :world :rotation-axis nil) real)
  (go? `(send *aero* :upper :torso :waist-p :joint-angle 30) real)
  (go? `(send *aero* :upper :torso :waist-p :joint-angle 20) real)
  (go? `(send *aero* :upper :arms :move-end-pos #f(300 0 200) :world :rotation-axis nil) real)
  (go? `(send *aero* :lower :rleg :move-end-pos #f(100 0 0) :world :rotation-axis nil) real)
  (go? `(send *aero* :move-waist #f(100 0 0)) real)
  (go? `(send *aero* :lower :rleg :move-end-pos #f(0 0 100) :world :rotation-axis nil) real)
  (go? `(send *aero* :lower :rleg :move-end-pos #f(0 0 -100) :world :rotation-axis nil) real)
  )
