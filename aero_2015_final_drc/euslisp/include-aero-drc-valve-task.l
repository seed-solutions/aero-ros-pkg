;; (load "package://aero_2015_final_drc/euslisp/aero-drc-testbed.l")

;; hand parameters
(defvar *hand-free-space* 40.0) ;; stable

;; valve parameters
(defvar *valve-radius* 130.0)
(defvar *grab-point* 115.0)

;; jsk-test-bed-1
;; (defvar *valve-radius* 130.0)
;; (defvar *grab-point* 115.0)

;; ;; jsk-test-bed-2
;; (defvar *valve-radius* 175.0)
;; (defvar *grab-point* 170.0)

;; variables
(defvar *left-grab-pose*)
(defvar *right-grab-pose*)


(defun setup-valve (&optional (x -1000) (y 0) (z 0) (valve-ang 0))
  (reset-env-to *drc-valve* (deg2rad -135))
  (init-testbed *drc-valve* :pos (float-vector x y z) :rot 0)
  (send (send *drc-valve* :joint :crank-joint) :joint-angle valve-ang)
  )

(defun release-arm (arm)
  (send *aero* :magic-open arm)
  (if (eq arm :larm)
      (send *aero* :upper arm :move-end-pos (float-vector 0 (* *hand-free-space* 2) 0) :local)
    (send *aero* :upper arm :move-end-pos (float-vector 0 (- (* *hand-free-space* 2)) 0) :local))
  )

(defun rotate-valve (arm grab-at-where rotate-how-much &optional (real *real*))
  ;; grab-at-where is valve degree you grabbed at, rotate-how-much is valve degree you rotate from current grab
  (send (send *drc-valve* :joint :crank-joint) :joint-angle 0)
  (send *aero* :translate
	(v+ (v- (send (send *drc-valve* :link :valve-handle-link) :worldpos)
		(send *aero* :upper arm :end-coords :worldpos))
	    (float-vector 0
			  (* *grab-point*
			     (sin (deg2rad (+ grab-at-where
					      (send (send *drc-valve* :joint :crank-joint)
						    :joint-angle)))))
			  (* *grab-point*
			     (cos (deg2rad (+ grab-at-where
					      (send (send *drc-valve* :joint :crank-joint)
						    :joint-angle))))))))
  (dotimes (i rotate-how-much)
    (setq hand-pos (float-vector 0
				 (* *grab-point*
				    (sin (deg2rad (+ grab-at-where
						     (send (send *drc-valve* :joint :crank-joint)
							   :joint-angle)))))
				 (* *grab-point*
				    (cos (deg2rad (+ grab-at-where
						     (send (send *drc-valve* :joint :crank-joint)
							   :joint-angle)))))
				 ))
    (send (send *drc-valve* :joint :crank-joint) :joint-angle
	  (+ (send (send *drc-valve* :joint :crank-joint) :joint-angle) 1))
    (setq hand-goal (float-vector 0
				  (* *grab-point*
				     (sin (deg2rad (+ grab-at-where
						      (send (send *drc-valve* :joint :crank-joint)
							    :joint-angle)))))
				  (* *grab-point*
				     (cos (deg2rad (+ grab-at-where
						      (send (send *drc-valve* :joint :crank-joint)
							    :joint-angle)))))
				  ))
    (if (or (eq (send *aero* :upper arm :move-end-pos (v- hand-goal hand-pos) :world) nil)
	    (eq (send *aero* :upper arm :move-end-rot -1 #f(1 0 0) :world) nil))
	(return-from rotate-valve i))
    (cond ((eq real t)
	   (send *ri* :angle-vector (send *aero* :stroke-vector) 20)
	   (unix::usleep (* 40 1000))))
    (if (boundp '*irtviewer*)
	(send *irtviewer* :draw-objects))
  ))
