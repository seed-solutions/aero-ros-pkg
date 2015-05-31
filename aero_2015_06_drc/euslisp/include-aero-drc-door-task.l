(load "package://aero_2015_06_drc/euslisp/aero-drc-testbed.l")

;; hand parameters
(defvar *arm* :larm) ;; stable
(defvar *narm* :rarm) ;; stable
(defvar *hand-free-space* 40.0) ;; stable

;; knob-parameters
(defvar *knob-x* 50)
(defvar *knob-y-pos* 770)
(defvar *knob-height* 900)
(defvar *grab-point* 55)
(defvar *open-angle* 20.0)

;; jsk-test-bed
;; (defvar *knob-x* 50)
;; (defvar *knob-y-pos* 740)
;; (defvar *knob-height* 900)
;; (defvar *grab-point* 55)
;; (defvar *open-angle* 60.0)

;; grab parameters
(defvar *grab-point-x* -10) ;; stable
(defvar *grab-point-z* -20) ;; stable


(defvar *exit-once* nil)

(defun setup (&optional (x -500) (y 500) (z 0))
  ; set door
  (send (send *door* :joint :door-hinge-joint) :joint-angle 0)
  (send (send *door* :joint :door-knob-joint) :joint-angle 0)
  ; set to initial position
  (init-testbed *door* :pos (float-vector x y z) :rot 0)
  (objects (list *door*))
  (send *aero* :draw-on)
  (setq *exit-once* nil)
  )

(defun reset-door ()
  (send (send *door* :joint :door-hinge-joint) :joint-angle 0)
  (send (send *door* :joint :door-knob-joint) :joint-angle 0)
  )

;; functions

(defun rotate-knob (&optional (power 0) (real *real*))
  (if (eq *exit-flag* t) (return-from rotate-knob nil))
  (let (hand-pos hand-goal)
    (setq power (if (< power 0) 0 power))
    (dotimes (i (round *open-angle*))
      (setq hand-pos (float-vector 0
				   (* *grab-point*
				      (- 1 (cos (deg2rad (send (send *door* :joint :door-knob-joint)
							       :joint-angle)))))
				   (- (* *grab-point*
					 (sin (abs (deg2rad (send (send *door* :joint :door-knob-joint)
								  :joint-angle))))))
				   ))
      (send (send *door* :joint :door-knob-joint) :joint-angle (- (+ i 1)))
      (setq hand-goal (float-vector 0
				    (* *grab-point*
				       (- 1 (cos (deg2rad (send (send *door* :joint :door-knob-joint)
								:joint-angle)))))
				    (- (* *grab-point*
					  (sin (abs (deg2rad (send (send *door* :joint :door-knob-joint)
								   :joint-angle))))))
				    ))
      (if (or (eq (send *aero* :upper *arm* :move-end-pos (v- hand-goal hand-pos) :world) nil)
	      (eq (send *aero* :upper *arm* :move-end-rot 1 #f(1 0 0) :world) nil)
	      (eq (send *aero* :upper *arm* :move-end-pos (float-vector 0 (- power) 0) :local) nil))
	  (progn (if (eq real t)
		     (go? `(send *ri* :angle-vector (send *aero* :stroke-vector))))
		 (return-from rotate-knob nil)))
      (send *irtviewer* :draw-objects)
      )
    (if (eq real t)
	(go? `(send *ri* :angle-vector (send *aero* :stroke-vector))))
    ))


(defun open-door (&optional (power 0) (real *real*))
  (if (eq *exit-flag* t) (return-from open-door nil))
  (let (hand-pos hand-goal)
    (setq power (if (< power 0) 0 power))
    (setq knob-offset (atan (/ (float *knob-x*) (- *knob-y-pos* *grab-point*))))
    (dotimes (i 89)
      (send (send *door* :joint :door-hinge-joint) :joint-angle (+ i 1))
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
      (if (or (eq (send *aero* :upper *arm* :move-end-pos (v- hand-goal hand-pos) :world) nil)
	      (eq (send *aero* :upper *arm* :move-end-rot -1 #f(0 0 1) :world) nil)
	      (eq (send *aero* :upper *arm* :move-end-pos (float-vector 0 0 (- power)) :local) nil))
	  (progn (if (eq real t)
		     (progn (send *irtviewer* :draw-objects)
			    (go? `(send *ri* :angle-vector (send *aero* :stroke-vector)))))
		 (go? `(send *aero* :magic-open *arm*))
		 (return-from open-door nil)))
      (send *irtviewer* :draw-objects)
      )))

;; debug

(defun for-test ()
  (go? `(send *aero* :hand-yaw-world *arm* (if (eq *arm* :larm) -90 90)))
  (go? `(send *aero* :hand *arm* :open-pose))
  (go? `(send *aero* :move-waist #f(0 0 -100)))
  (go? `(send *aero* :upper *arm* :move-end-pos #f(0 10 0) :world))
  (go? `(send *aero* :upper *arm* :move-end-pos #f(20 0 0) :world))
  (go? `(send *aero* :move-waist #f(0 0 -20)))
  (go? `(send *aero* :magic-close *arm*))
  (go? `(rotate-knob) nil)
  (go? `(open-door) nil)
  )

;; model samples

(defun sample-v0.02 ()
  (let (ui)
    (cond ((eq *exit-once* nil)
	   (go? `(send *aero* :hand-yaw-world *arm* (if (eq *arm* :larm) -90 90)))
	   (go? `(send *aero* :hand *arm* :open-pose))
	   ))
    (block :command-line
	   (dotimes (i 1000)
	     (print "enter command or nil to exit")
	     (setq ui (read-line))
	     (if (equal ui "exit-once")
		 (progn (setq *exit-once* t)
			(return-from sample-v0.02)))
	     (if (equal ui "nil")
		 (return-from :command-line)
	       (go? (read-from-string ui))))
	   )
    (setq diff-to-door-knob (v- (send (send *door* :link :door-knob) :worldpos)
				(send *aero* :upper *arm* :end-coords :worldpos)))
    (go? `(send *aero* :translate (v+ diff-to-door-knob
				      (float-vector (- *grab-point-x* *knob-x*)
						    (- *grab-point*)
						    *grab-point-z*))) nil)
    (go? `(rotate-knob) nil)
    (go? `(open-door) nil)
    ))
