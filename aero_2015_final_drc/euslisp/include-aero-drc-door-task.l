;; (load "package://aero_2015_final_drc/euslisp/aero-drc-testbed.l")

;; hand parameters
(defvar *hand-free-space* 40.0) ;; stable

;; knob-parameters
(defvar *knob-x* 50)
(defvar *knob-y-pos* 770)
(defvar *knob-height* 900)
(defvar *grab-point* 55)
(defvar *open-angle* 50.0)

;; grab parameters
(defvar *grab-point-x* -10) ;; stable
(defvar *grab-point-z* -20) ;; stable


;; virtual-door
(defvar *door-hinge-joint* 0)
(defvar *door-knob-joint* 0)

(defun setup-door (&optional (x -500) (y 500) (z 0))
  (setq *door-hinge-joint* 0)
  (setq *door-knob-joint* 0)
  )

;; functions

(defun rotate-knob (arm &optional (power 0) (open-angle *open-angle*) (real *real*))
  (let (hand-pos hand-goal)
    (setq power (if (< power 0) 0 power))
    (dotimes (i (round open-angle))
      (setq hand-pos (float-vector 0
				   (* *grab-point*
				      (- 1 (cos (deg2rad *door-knob-joint*))))
				   (- (* *grab-point*
					 (sin (abs (deg2rad *door-knob-joint*)))))
				   ))
      (setq *door-knob-joint* (- (+ i 1)))
      (setq hand-goal (float-vector 0
				    (* *grab-point*
				       (- 1 (cos (deg2rad *door-knob-joint*))))
				    (- (* *grab-point*
					  (sin (abs (deg2rad *door-knob-joint*)))))
				    ))
      (if (or (eq (send *aero* :upper arm :move-end-pos (v- hand-goal hand-pos) :world) nil)
	      (eq (send *aero* :upper arm :move-end-rot 1 #f(1 0 0) :world) nil)
	      (eq (send *aero* :upper arm :move-end-pos (float-vector 0 (- power) 0) :local) nil))
	  (progn (if (eq real t)
		     (send *ri* :angle-vector (send *aero* :stroke-vector)))
		 (return-from rotate-knob nil)))
      (if (boundp '*irtviewer*)
	  (send *irtviewer* :draw-objects))
      )
    (if (eq real t)
	(send *ri* :angle-vector (send *aero* :stroke-vector)))
    ))


(defun open-door (arm &optional (power 0) (real *real*))
  (let (hand-pos hand-goal)
    (setq power (if (< power 0) 0 power))
    (setq knob-offset (atan (/ (float *knob-x*) (- *knob-y-pos* *grab-point*))))
    (dotimes (i 89)
      (setq *door-hinge-joint* (+ i 1))
      (setq hand-pos (float-vector (* (- *knob-y-pos* *grab-point*)
				      (sin (- (deg2rad i) knob-offset)))
				   (* (- *knob-y-pos* *grab-point*)
				      (cos (- (deg2rad i) knob-offset)))
				   0))
      (setq hand-goal (float-vector (* (- *knob-y-pos* *grab-point*)
				       (sin (- (deg2rad (+ i 1)) knob-offset)))
				    (* (- *knob-y-pos* *grab-point*)
				       (cos (- (deg2rad (+ i 1)) knob-offset)))
				    0))
      (if (or (eq (send *aero* :upper arm :move-end-pos (v- hand-goal hand-pos) :world) nil)
	      (eq (send *aero* :upper arm :move-end-rot -1 #f(0 0 1) :world) nil)
	      (eq (send *aero* :upper arm :move-end-pos (float-vector 0 0 (- power)) :local) nil))
	  (progn (if (eq real t)
		     (send *ri* :angle-vector (send *aero* :stroke-vector)))
		 (send *aero* :magic-open arm)
		 (return-from open-door nil)))
      (if (boundp '*irtviewer*)
	  (send *irtviewer* :draw-objects))
      )))
