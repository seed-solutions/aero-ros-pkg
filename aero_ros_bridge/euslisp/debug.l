(load "package://aero_ros_bridge/euslisp/aero-upper-interface.l")
(load "package://aero_ros_bridge/euslisp/aero-lower-interface.l")

(defmethod aero-upper-interface
  (:stroke-to-angle (stroke)
     (let ((neck-stroke-A (cdr (assoc "neck_right_joint" stroke :test #'equal)))
	   (neck-stroke-B (cdr (assoc "neck_left_joint" stroke :test #'equal)))
	   (waist-stroke-A (cdr (assoc "waist_right_joint" stroke :test #'equal)))
	   (waist-stroke-B (cdr (assoc "waist_left_joint" stroke :test #'equal)))
	   neck-pitch-stroke
	   neck-roll-stroke
	   waist-pitch-stroke
	   waist-roll-stroke
	   (neck-roll-direction 1)
	   (waist-roll-direction 1)
	   (left-wrist-stroke-A (cdr (assoc "l_wrist_top_joint" stroke :test #'equal)))
	   (left-wrist-stroke-B (cdr (assoc "l_wrist_bottom_joint" stroke :test #'equal)))
	   left-wrist-roll-stroke
	   left-wrist-pitch-stroke
	   (right-wrist-stroke-A (cdr (assoc "r_wrist_top_joint" stroke :test #'equal)))
	   (right-wrist-stroke-B (cdr (assoc "r_wrist_bottom_joint" stroke :test #'equal)))
	   right-wrist-roll-stroke
	   right-wrist-pitch-stroke
	   (left-wrist-pitch-direction 1)
	   (left-wrist-roll-direction 1)
	   (right-wrist-pitch-direction 1)
	   (right-wrist-roll-direction 1)
	   )
       (setq neck-pitch-stroke (/ (+ neck-stroke-A neck-stroke-B) 2))
       (setq neck-roll-stroke (- neck-stroke-A neck-pitch-stroke))
       (setq waist-pitch-stroke (/ (+ waist-stroke-A waist-stroke-B) 2))
       (setq waist-roll-stroke (- waist-stroke-A waist-pitch-stroke))
       (setq left-wrist-roll-stroke (/ (+ left-wrist-stroke-A left-wrist-stroke-B) 2))
       (setq left-wrist-pitch-stroke (- left-wrist-stroke-A left-wrist-roll-stroke))
       (setq right-wrist-roll-stroke (/ (+ right-wrist-stroke-A right-wrist-stroke-B) 2))
       (setq right-wrist-pitch-stroke (- right-wrist-stroke-A right-wrist-roll-stroke))
       (if (> neck-stroke-A neck-stroke-B)
	   (setq neck-roll-direction -1) (setq neck-roll-direction 1))
       (if (> waist-stroke-A waist-stroke-B)
	   (setq wasit-roll-direction 1) (setq waist-roll-direction -1))
       (if (> left-wrist-stroke-A left-wrist-stroke-B)
	   (setq left-wrist-pitch-direction -1) (setq left-wrist-pitch-direction 1))
       (if (>= left-wrist-roll-stroke 0)
	   (setq left-wrist-roll-direction -1) (setq left-wrist-roll-direction 1))
       (if (> right-wrist-stroke-A right-wrist-stroke-B)
	   (setq right-wrist-pitch-direction -1) (setq right-wrist-pitch-direction 1))
       (if (>= right-wrist-roll-stroke 0)
	   (setq right-wrist-roll-direction -1) (setq right-wrist-roll-direction 1))
       (float-vector (- (send self :rev-shoulder-p-table (cdr (assoc "r_shoulder_pitch_joint" stroke :test #'equal))))
		     (- (send self :rev-shoulder-r-table (cdr (assoc "r_shoulder_roll_joint" stroke :test #'equal))))
		     (cdr (assoc "r_elbow_yaw_joint" stroke :test #'equal))
		     (- (send self :rev-elbow-p-table (cdr (assoc "r_elbow_pitch_joint" stroke :test #'equal))))
		     (cdr (assoc "r_wrist_roll_joint" stroke :test #'equal))
		     (* right-wrist-pitch-direction (send self :rev-wrist-p-table (abs right-wrist-pitch-stroke)))
		     (* right-wrist-roll-direction (send self :rev-wrist-r-table (- (abs right-wrist-roll-stroke))))
		     (cdr (assoc "r_hand_joint" stroke :test #'equal))
		     0
		     0
		     0
		     (- (send self :rev-shoulder-p-table (cdr (assoc "l_shoulder_pitch_joint" stroke :test #'equal))))
		     (send self :rev-shoulder-r-table (cdr (assoc "l_shoulder_roll_joint" stroke :test #'equal)))
		     (cdr (assoc "l_elbow_yaw_joint" stroke :test #'equal))
		     (- (send self :rev-elbow-p-table (cdr (assoc "l_elbow_pitch_joint" stroke :test #'equal))))
		     (cdr (assoc "l_wrist_roll_joint" stroke :test #'equal))
		     (* left-wrist-pitch-direction (send self :rev-wrist-p-table (abs left-wrist-pitch-stroke)))
		     (* left-wrist-roll-direction (send self :rev-wrist-r-table (- (abs left-wrist-roll-stroke))))
		     (cdr (assoc "r_hand_joint" stroke :test #'equal))
		     0
		     0
		     0
		     (cdr (assoc "waist_pitch_joint" stroke :test #'equal))
		     (send self :rev-waist-p-table waist-pitch-stroke)
		     (* waist-roll-direction (send self :rev-waist-r-table (abs waist-roll-stroke)))
		     (cdr (assoc "neck_yaw_joint" stroke :test #'equal))
		     (send self :rev-neck-p-table neck-pitch-stroke)
		     (* neck-roll-direction (send self :rev-neck-r-table (abs neck-roll-stroke)))
		     )))

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
			      "neck_left_joint"))
	   (i 0)
	   (result nil))
       (dolist (x joint-names)
       	 (setq result (cons (cons x  (elt stroke-vector i)) result))
       	 (setq i (+ i 1)))
       result))
)

(aero-upper-init)
(aero-lower-init)
(send *aero-upper* :reset-manip-pose)
(objects (list *aero-upper* *aero-lower*))
(send *aero-upper* :fix-upper-to-coords)

(defun debug-reset ()
  (send *aero-upper* :reset-pose)
  (send *aero-lower* :rarm :shoulder-y :joint-angle -30)
  (send *aero-lower* :larm :shoulder-y :joint-angle 30)
  (send *aero-lower* :rleg :crotch-y :joint-angle 30)
  (send *aero-lower* :lleg :crotch-y :joint-angle -30)
  (send *aero-lower* :rarm :elbow-p :joint-angle 45)
  (send *aero-lower* :larm :elbow-p :joint-angle 45)
  (send *aero-lower* :lleg :knee-p :joint-angle 45)
  (send *aero-lower* :rleg :knee-p :joint-angle 45)
  (send *aero-upper* :rarm :elbow-p :joint-angle -90)
  (send *aero-upper* :larm :elbow-p :joint-angle -90)
  )

(defun stroke-vector-test ()
  (debug-reset)
  (load "aero-lower-strokes.l")
  (send *aero-upper* :stroke-vector-all)
  )

(defun av-v-test ()
  (format nil "~A ~A" (send *aero-upper* :angle-vector)
	  (send *ri* :stroke-to-angle (send *ri* :debug-stroke (send *aero-upper* :stroke-vector))))
  )

(defun angle-vector-test ()
  (send *aero-upper* :angle-vector (send *ri* :actual-vector))
  )
