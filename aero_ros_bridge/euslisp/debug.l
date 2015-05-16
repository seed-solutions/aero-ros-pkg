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
  (send *aero* :reset-pose)
  (send *aero* :lower :rarm :shoulder-y :joint-angle -45)
  (send *aero* :lower :larm :shoulder-y :joint-angle 45)
  (send *aero* :lower :rleg :crotch-y :joint-angle 45)
  (send *aero* :lower :lleg :crotch-y :joint-angle -45)
  (send *aero* :lower :rarm :elbow-p :joint-angle 45)
  (send *aero* :lower :larm :elbow-p :joint-angle 45)
  (send *aero* :lower :lleg :knee-p :joint-angle 45)
  (send *aero* :lower :rleg :knee-p :joint-angle 45)
  (send *aero* :upper :rarm :elbow-p :joint-angle -90)
  (send *aero* :upper :larm :elbow-p :joint-angle -90)
  )

(defun stroke-vector-test ()
  (debug-reset)
  (send *aero* :stroke-vector)
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
