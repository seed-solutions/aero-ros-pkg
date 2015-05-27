(load "package://aero_2015_06_drc/euslisp/aero-drc-testbed.l")

(defvar *arm* :larm)
(defvar *narm* :rarm)
(defvar *knob-x* 50)
(defvar *knob-y-pos* 770)
(defvar *knob-height* 900)
(defvar *grab-point* 55)
(defvar *hand-free-space* 40.0)
(defvar *open-angle* 50.0)

(defun setup (&optional (x -500) (y 500) (z 0))
  ; set door
  (send (send *door* :joint :door-hinge-joint) :joint-angle 0)
  (send (send *door* :joint :door-knob-joint) :joint-angle 0)
  ; set to initial position
  (init-testbed *door* :pos (float-vector x y z) :rot 0)
  (objects (list *door*))
  (send *aero* :draw-on)
  )

;; functions

(defun rotate-knob ()
  (if (eq *exit-flag* t) (return-from rotate-knob nil))
  (send *aero* :upper *arm* :wrist-y :joint-angle
	(if (eq *arm* :larm)
	    (- (send *aero* :upper *arm* :wrist-y :joint-angle) *open-angle*)
	  (+ (send *aero* :upper *arm* :wrist-y :joint-angle) *open-angle*)))
  (send (send *door* :link :door-knob) :rotate (* *open-angle* (/ pi 180)) #f(1 0 0) :world)
  (go? `(send *aero* :upper *arm* :move-end-pos
	      (float-vector
	       0 
	       (* *grab-point* (- 1 (cos (* *open-angle* (/ pi 180)))))
	       (- (abs (* *grab-point* (sin (* *open-angle* (/ pi 180)))))))
	      :world))
  )

(defun open-door (&optional (real *real*))
  (if (eq *exit-flag* t) (return-from open-door nil))
  (setq knob-offset (atan (/ (float *knob-x*) (- *knob-y-pos* *grab-point*))))
  (dotimes (i 89)
    (send (send *door* :joint :door-hinge-joint) :joint-angle (+ i 1))
    (setq hand-pos (send (send (send *aero* :upper :larm :end-coords)
			       :copy-worldcoords) :pos))
    (setq hand-goal (float-vector (* (- *knob-y-pos* *grab-point*)
				     (sin (- (* (+ i 1) (/ pi 180)) knob-offset)))
				  (* (- *knob-y-pos* *grab-point*)
				     (cos (- (* (+ i 1) (/ pi 180)) knob-offset)))
				  (aref hand-pos 2)))
    (send *aero* :upper *arm* :wrist-p :joint-angle
	  (- (send *aero* :upper *arm* :wrist-p :joint-angle) 5))
    (if (eq (send *aero* :upper *arm* :move-end-pos (v- hand-goal hand-pos) :world) nil)
	(progn (send *aero* :upper *arm* :wrist-p :joint-angle
		     (+ (send *aero* :upper *arm* :wrist-p :joint-angle) 5))
	       (if (eq real t)
		   (progn (send *irtviewer* :draw-objects)
			  (go? `(send *ri* :angle-vector (send *aero* :stroke-vector)))))
			  ;; (send *ri* :angle-vector (send *aero* :stroke-vector))
			  ;; (unix::usleep (* 5000 1000))))
	       (go? `(send *aero* :magic-open *arm*))
	       (return-from open-door nil)))
    (send *irtviewer* :draw-objects)
    ))      

(defun sample-v0.02 ()
  (let (ui)
    (go? `(send *aero* :hand-yaw-world *arm* (if (eq *arm* :larm) -90 90)))
    (go? `(send *aero* :hand *arm* :open-pose))
    (block :command-line
	   (dotimes (i 1000)
	     (print "enter command or nil to exit")
	     (setq ui (read-line))
	     (if (equal ui "nil")
		 (return-from :command-line)
	       (go? (read-from-string ui))))
	   )
    (setq diff-to-door-knob (v- (send (send *door* :link :door-knob) :worldpos)
				(send *aero* :upper *arm* :end-coords :worldpos)))
    (go? `(send *aero* :translate (v+ diff-to-door-knob
				      (float-vector (- *hand-free-space*) (- *grab-point*) 0))) nil)
    (rotate-knob)
    (go? `(open-door) nil)
    ))
