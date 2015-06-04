;; (load "package://aero_2015_final_drc/euslisp/aero-drc-testbed.l")

;; hand parameters
(defvar *hand-free-space* 40.0) ;; stable

;; valve parameters
(defvar *valve-radius* 250.0)
(defvar *grab-point* 222.0)
(defvar *valve-height* 1110.0)

;; variables
(defvar *left-grab-pose*)
(defvar *right-grab-pose*)


(defun setup-valve (&optional (x -833) (y 0) (z 0) (valve-ang 0)) ;; -480(from valve top)
  )

(defun release-arm (arm)
  (send *aero* :magic-open arm)
  (if (eq arm :larm)
    ;;   (send *aero* :upper arm :move-end-pos (float-vector 0 (* *hand-free-space* 2) 0) :local)
    ;; (send *aero* :upper arm :move-end-pos (float-vector 0 (- (* *hand-free-space* 2)) 0) :local))
      (send *aero-upper* arm :inverse-kinematics
	    (send (send (send *aero-upper* arm :end-coords) :copy-worldcoords)
		  :translate (float-vector 0 (* *hand-free-space* 2) 0) :local)
	    :link-list (send *aero-upper* :link-list
			     (send *aero-upper* arm :end-coords :parent)))
    (send *aero-upper* arm :inverse-kinematics
	  (send (send (send *aero-upper* arm :end-coords) :copy-worldcoords)
		:translate (float-vector 0 (- (* *hand-free-space* 2)) 0) :local)
	  :link-list (send *aero-upper* :link-list
			   (send *aero-upper* arm :end-coords :parent))))
  )

(defun rotate-valve (arm grab-at-where rotate-how-much &optional (real *real*))
  ;; grab-at-where is valve degree you grabbed at, rotate-how-much is valve degree you rotate from current grab
  (let ((crank-joint-angle 0))
    (dotimes (i rotate-how-much)
      (setq hand-pos (float-vector 0
				   (* *grab-point*
				      (sin (deg2rad (+ grab-at-where
						       crank-joint-angle))))
				   (* *grab-point*
				      (cos (deg2rad (+ grab-at-where
						       crank-joint-angle))))
				   ))
      (incf crank-joint-angle)
      (setq hand-goal (float-vector 0
				    (* *grab-point*
				       (sin (deg2rad (+ grab-at-where
							crank-joint-angle))))
				    (* *grab-point*
				       (cos (deg2rad (+ grab-at-where
							crank-joint-angle))))
				    ))
    ;; (if (or (eq (send *aero* :upper arm :move-end-pos (v- hand-goal hand-pos) :world) nil)
    (if (or (eq (send *aero-upper* arm :inverse-kinematics
		      (send (send (send *aero-upper* arm :end-coords) :copy-worldcoords)
			    :translate (v- hand-goal hand-pos) :world)
		      :link-list (send *aero-upper* :link-list
				       (send *aero-upper* arm :end-coords :parent))) nil)
	    (eq (send *aero* :upper arm :move-end-rot -1 #f(1 0 0) :world) nil))
	(return-from rotate-valve i))
    (cond ((eq real t)
	   (send *ri* :angle-vector (send *aero* :stroke-vector) 20)
	   (unix::usleep (* 40 1000))))
    (if (boundp '*irtviewer*)
	(send *irtviewer* :draw-objects))
  ))
