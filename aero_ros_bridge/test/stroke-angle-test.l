(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(load "package://aero_ros_bridge/euslisp/aero-interface.l")

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
         (setq result (cons (cons x (elt stroke-vector i)) result))
         (setq i (+ i 1)))
       result))
)

(aero-init)
(send *aero* :draw-on)

;; tmp
(setq ((send *aero* :upper :torso :waist-p) . max-angle) 40)
(setq ((send *aero* :upper :rarm :elbow-p) . max-angle) -1)
;; tmp

(defun test-stroke-vector (&key (step 0.1))
  (let (ja)
    (send *aero* :reset-manip-pose)
    (dolist (x (send *aero* :upper :joint-list))
      (block continue
	     #| don't test yaw joint |#
	     (if (or (equal (send x :min-angle) (- *inf*))
		     (equal (send x :min-angle) nil)
		     (equal (send x :max-angle) *inf*)
		     (equal (send x :max-angle) nil))
		 (return-from continue))
	     #| don't test mirror arm |#
	     (if (substringp "larm" (send x :name))
		 (return-from continue))
	     (setq ja (send x :min-angle))
	     (while (< ja (send x :max-angle))
	       (send x :joint-angle ja)
	       (send *irtviewer* :draw-objects)
	       (send *aero* :stroke-vector)
	       (setq ja (+ ja step)))
	     ))
    (dolist (x (send *aero* :lower :joint-list))
      (block continue
	     (if (or (equal (send x :min-angle) (- *inf*))
		     (equal (send x :min-angle) nil)
		     (equal (send x :max-angle) *inf*)
		     (equal (send x :max-angle) nil))
		 (return-from continue))
	     #| don't test mirror legs |#
	     (if (or (substringp "rarm" (send x :name))
		     (substringp "lleg" (send x :name))
		     (substringp "larm" (send x :name)))
		 (return-from continue))
	     (setq ja (send x :min-angle))
	     (while (< ja (send x :max-angle))
	       (send x :joint-angle ja)
	       (send *irtviewer* :draw-objects)
	       (send *aero* :stroke-vector)
	       (setq ja (+ ja step)))
	     ))
    t))

(defun test-angle-vector (&key (step 0.1))
  (let (ja)
    (send *aero* :reset-manip-pose)
    (dolist (x (send *aero* :upper :joint-list))
      (block continue
	     #| don't test yaw joint |#
	     (if (or (equal (send x :min-angle) (- *inf*))
		     (equal (send x :min-angle) nil)
		     (equal (send x :max-angle) *inf*)
		     (equal (send x :max-angle) nil))
		 (return-from continue))
	     #| don't test mirror arm |#
	     (if (substringp "larm" (send x :name))
		 (return-from continue))
	     (setq ja (send x :min-angle))
	     (while (< ja (send x :max-angle))
	       (send x :joint-angle ja)
	       (send *irtviewer* :draw-objects)
	       (send *ri* :stroke-to-angle
		     (send *ri* :debug-stroke (send *aero* :stroke-vector)))
	       (setq ja (+ ja step)))
	     ))
    (dolist (x (send *aero* :lower :joint-list))
      (block continue
	     (if (or (equal (send x :min-angle) (- *inf*))
		     (equal (send x :min-angle) nil)
		     (equal (send x :max-angle) *inf*)
		     (equal (send x :max-angle) nil))
		 (return-from continue))
	     #| don't test mirror legs |#
	     (if (or (substringp "rarm" (send x :name))
		     (substringp "lleg" (send x :name))
		     (substringp "larm" (send x :name)))
		 (return-from continue))
	     (setq ja (send x :min-angle))
	     (while (< ja (send x :max-angle))
	       (send x :joint-angle ja)
	       (send *irtviewer* :draw-objects)
	       (send *ri* :stroke-to-angle
		     (send *ri* :debug-stroke (send *aero* :stroke-vector)))
	       (setq ja (+ ja step)))
	     ))
    t))

(defun test-av-v (&key (step 0.1))
  (let (ja)
    (send *aero* :reset-manip-pose)
    (dolist (x (send *aero* :upper :joint-list))
      (block continue
	     #| don't test yaw joint |#
	     (if (or (equal (send x :min-angle) (- *inf*))
		     (equal (send x :min-angle) nil)
		     (equal (send x :max-angle) *inf*)
		     (equal (send x :max-angle) nil))
		 (return-from continue))
	     #| don't test mirror arm |#
	     (if (substringp "larm" (send x :name))
		 (return-from continue))
	     (setq ja (send x :min-angle))
	     (while (< ja (send x :max-angle))
	       (send x :joint-angle ja)
	       (send *irtviewer* :draw-objects)
	       (if (not (eps-v=
			 (car (send *aero* :angle-vector))
			 (car (send *ri* :stroke-to-angle
				    (send *ri* :debug-stroke
					  (send *aero* :stroke-vector)))) 0.1))
		   (progn (warn "~S ~S~%" (car (send *aero* :angle-vector))
				(car (send *ri* :stroke-to-angle
					   (send *ri* :debug-stroke
						 (send *aero* :stroke-vector)))))
			  (return-from test-av-v nil)))
	       (setq ja (+ ja step)))
	     ))
    (dolist (x (send *aero* :lower :joint-list))
      (block continue
	     (if (or (equal (send x :min-angle) (- *inf*))
		     (equal (send x :min-angle) nil)
		     (equal (send x :max-angle) *inf*)
		     (equal (send x :max-angle) nil))
		 (return-from continue))
	     #| don't test mirror legs |#
	     (if (or (substringp "rarm" (send x :name))
		     (substringp "lleg" (send x :name))
		     (substringp "larm" (send x :name)))
		 (return-from continue))
	     (setq ja (send x :min-angle))
	     (while (< ja (send x :max-angle))
	       (send x :joint-angle ja)
	       (send *irtviewer* :draw-objects)
	       (if (not (eps-v=
			 (cadr (send *aero* :angle-vector))
			 (cadr (send *ri* :stroke-to-angle
				     (send *ri* :debug-stroke
					   (send *aero* :stroke-vector)))) 0.1))
		   (progn (warn "~S ~S~%" (cadr (send *aero* :angle-vector))
				(cadr (send *ri* :stroke-to-angle
					    (send *ri* :debug-stroke
						  (send *aero* :stroke-vector)))))
			  (return-from test-av-v nil)))
	       (setq ja (+ ja step)))
	     ))
    t))

#| if test-angle-vector-1 fails, first test test-stroke-vector-1 |#

;; (deftest test-stroke-vector-1
;;   (assert (test-stroke-vector :step 1)))

#| if test-av-v-1 fails, first test test-angle-vector-1 |#

;; (deftest test-angle-vector-1
;;   (assert (test-angle-vector :step 1)))

(deftest test-av-v-1
  (assert (test-av-v :step 1)))

(run-all-tests)
(exit)
