(load "package://aero_2015_06_drc/euslisp/aero-drc-testbed.l")

(defvar *arm* :larm)
(defvar *narm* :rarm)
(defvar *valve-radius* 130.0)
(defvar *hand-free-space* 40.0)
(defvar *grab-point* 115.0)

(defvar *exit-once* nil)

(defun setup (&optional (x -1000) (y 0) (z 0) (valve-ang 0))
					; set to initial position
  (init-testbed *drc-valve* :pos (float-vector x y z) :rot 0)
  (send (send *drc-valve* :joint :crank-joint) :joint-angle valve-ang)
  (objects (list *drc-valve*))
  (send *aero* :draw-on)
  (setq *exit-once* nil)
  )

;; functions

(defun rotate-valve (&optional (real *real*))
  (if (eq *exit-flag* t) (return-from rotate-valve nil))
  (dotimes (i 89)
    (setq hand-pos (float-vector 0
				 (* *grab-point*
				    (sin (deg2rad (+ 45 (send (send *drc-valve* :joint :crank-joint)
							      :joint-angle)))))
				 (* *grab-point*
				    (cos (deg2rad (+ 45 (send (send *drc-valve* :joint :crank-joint)
							      :joint-angle)))))
				 ))
    (send (send *drc-valve* :joint :crank-joint) :joint-angle (+ i 1))
    (setq hand-goal (float-vector 0
				  (* *grab-point*
				     (sin (deg2rad (+ 45 (send (send *drc-valve* :joint :crank-joint)
								  :joint-angle)))))
				  (* *grab-point*
				     (cos (deg2rad (+ 45 (send (send *drc-valve* :joint :crank-joint)
							       :joint-angle)))))
				  ))
    (if (or (eq (send *aero* :upper *arm* :move-end-pos (v- hand-goal hand-pos) :world) nil)
	    (eq (send *aero* :upper *arm* :move-end-rot -1 #f(1 0 0) :world) nil))
	(return-from rotate-valve nil))
    (cond ((eq real t)
	   (send *ri* :angle-vector (send *aero* :stroke-vector) 100)
	   (unix::usleep (* 100 1000))))
    (send *irtviewer* :draw-objects))
  )

;; model samples

(defun sample-v0.02 ()
  (let (ui rot)
    (cond ((eq *exit-once* nil)
	   (go? `(send *aero* :hand-yaw-world *arm*
		       (if (eq *arm* :larm) (- (- 45 (send (send *drc-valve* :joint :crank-joint)
							   :joint-angle))) 
			 (- 45 (send (send *drc-valve* :joint :crank-joint)
				     :joint-angle)))))
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
    (setq diff-to-valve (v- (send (send *drc-valve* :link :valve-handle-link) :worldpos)
    			    (send *aero* :upper *arm* :end-coords :worldpos)))
    (go? `(send *aero* :translate (v+ diff-to-valve
                                      (float-vector 0
    						    (* *grab-point*
    						       (sin (deg2rad (+ 45 (send (send *drc-valve* :joint :crank-joint)
    										 :joint-angle)))))
    						    (* *grab-point*
    						       (cos (deg2rad (+ 45 (send (send *drc-valve* :joint :crank-joint)
    										 :joint-angle))))))))
    	 nil)
    (go? `(rotate-valve) nil)
    ))


;; real robot logs

