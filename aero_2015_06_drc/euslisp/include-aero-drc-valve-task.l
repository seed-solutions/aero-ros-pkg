(load "package://aero_2015_06_drc/euslisp/aero-drc-testbed.l")

;; hand parameters
(defvar *arm* :larm) ;; stable
(defvar *narm* :rarm) ;; stable
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


(defvar *exit-once* 0)

(defun setup (&optional (x -1000) (y 0) (z 0) (valve-ang 0))
  (init-testbed *drc-valve* :pos (float-vector x y z) :rot 0)
  (send (send *drc-valve* :joint :crank-joint) :joint-angle valve-ang)
  (objects (list *drc-valve*))
  (send *aero* :draw-on)
  (if (not (eq *arm* :larm)) (go? `(switch-arms)))  
  (setq *exit-once* 0)
  )

;; functions

(defun switch-arms ()
  (let (tmp)
    (setq tmp *arm*)
    (setq *arm* *narm*)
    (setq *narm* tmp)
    ))

(defun save-pose (arm)
  (if (eq arm :larm)
      (setq *left-grab-pose*
	    (list (send *aero* :upper arm :shoulder-p :joint-angle)
		  (send *aero* :upper arm :shoulder-r :joint-angle)
		  (send *aero* :upper arm :shoulder-y :joint-angle)
		  (send *aero* :upper arm :elbow-p :joint-angle)
		  (send *aero* :upper arm :wrist-y :joint-angle)
		  (send *aero* :upper arm :wrist-p :joint-angle)
		  (send *aero* :upper arm :wrist-r :joint-angle)
		  (send *aero* :upper :torso :waist-p :joint-angle)
		  (send *aero* :upper :torso :waist-r :joint-angle)
		  (send *aero* :upper :torso :waist-y :joint-angle)))
    (setq *right-grab-pose*
	  (list (send *aero* :upper arm :shoulder-p :joint-angle)
		(send *aero* :upper arm :shoulder-r :joint-angle)
		(send *aero* :upper arm :shoulder-y :joint-angle)
		(send *aero* :upper arm :elbow-p :joint-angle)
		(send *aero* :upper arm :wrist-y :joint-angle)
		(send *aero* :upper arm :wrist-p :joint-angle)
		(send *aero* :upper arm :wrist-r :joint-angle)
		(send *aero* :upper :torso :waist-p :joint-angle)
		(send *aero* :upper :torso :waist-r :joint-angle)
		(send *aero* :upper :torso :waist-y :joint-angle)))
    ))

(defun load-pose (arm)
  (cond ((eq arm :larm)
	 (send *aero* :upper arm :shoulder-p :joint-angle (elt *left-grab-pose* 0))
	 (send *aero* :upper arm :shoulder-r :joint-angle (elt *left-grab-pose* 1))
	 (send *aero* :upper arm :shoulder-y :joint-angle (elt *left-grab-pose* 2))
	 (send *aero* :upper arm :elbow-p :joint-angle (elt *left-grab-pose* 3))
	 (send *aero* :upper arm :wrist-y :joint-angle (elt *left-grab-pose* 4))
	 (send *aero* :upper arm :wrist-p :joint-angle (elt *left-grab-pose* 5))
	 (send *aero* :upper arm :wrist-r :joint-angle (elt *left-grab-pose* 6))
	 (send *aero* :upper :torso :waist-p :joint-angle (elt *left-grab-pose* 7))
	 (send *aero* :upper :torso :waist-r :joint-angle (elt *left-grab-pose* 8))
	 (send *aero* :upper :torso :waist-y :joint-angle (elt *left-grab-pose* 9)))
	((eq arm :rarm)
	 (send *aero* :upper arm :shoulder-p :joint-angle (elt *right-grab-pose* 0))
	 (send *aero* :upper arm :shoulder-r :joint-angle (elt *right-grab-pose* 1))
	 (send *aero* :upper arm :shoulder-y :joint-angle (elt *right-grab-pose* 2))
	 (send *aero* :upper arm :elbow-p :joint-angle (elt *right-grab-pose* 3))
	 (send *aero* :upper arm :wrist-y :joint-angle (elt *right-grab-pose* 4))
	 (send *aero* :upper arm :wrist-p :joint-angle (elt *right-grab-pose* 5))
	 (send *aero* :upper arm :wrist-r :joint-angle (elt *right-grab-pose* 6))
	 (send *aero* :upper :torso :waist-p :joint-angle (elt *left-grab-pose* 7))
	 (send *aero* :upper :torso :waist-r :joint-angle (elt *left-grab-pose* 8))
	 (send *aero* :upper :torso :waist-y :joint-angle (elt *left-grab-pose* 9)))
      ))

(defun copy-arm-pose (copy-from copy-to)
  (send *aero* :upper copy-to :shoulder-p :joint-angle
	(send *aero* :upper copy-from :shoulder-p :joint-angle))
  (send *aero* :upper copy-to :shoulder-r :joint-angle
	(- (send *aero* :upper copy-from :shoulder-r :joint-angle)))
  (send *aero* :upper copy-to :shoulder-y :joint-angle
	(- (send *aero* :upper copy-from :shoulder-y :joint-angle)))
  (send *aero* :upper copy-to :elbow-p :joint-angle
	(send *aero* :upper copy-from :elbow-p :joint-angle))
  (send *aero* :upper copy-to :wrist-y :joint-angle
	(- (send *aero* :upper copy-from :wrist-y :joint-angle)))
  (send *aero* :upper copy-to :wrist-p :joint-angle
	(send *aero* :upper copy-from :wrist-p :joint-angle))
  (send *aero* :upper copy-to :wrist-r :joint-angle
	(- (send *aero* :upper copy-from :wrist-r :joint-angle)))
  (send *aero* :hand copy-to :joint-angle
	(send *aero* :hand copy-from :joint-angle))
  )

(defun release-arm (arm)
  (go? `(send *aero* :magic-open *arm*))
  (if (eq arm :larm)
      (go? `(send *aero* :upper *arm* :move-end-pos (float-vector 0 (* *hand-free-space* 2) 0) :local))
    (go? `(send *aero* :upper *arm* :move-end-pos (float-vector 0 (- (* *hand-free-space* 2)) 0) :local)))
  )

(defun rotate-valve (arm grab-at-where rotate-how-much &optional (real *real*))
  ;; grab-at-where is valve degree you grabbed at, rotate-how-much is valve degree you rotate from current grab
  (if (eq *exit-flag* t) (return-from rotate-valve nil))
  (send (send *drc-valve* :joint :crank-joint) :joint-angle 0)
  (send *aero* :translate
	(v+ (v- (send (send *drc-valve* :link :valve-handle-link) :worldpos)
		(send *aero* :upper *arm* :end-coords :worldpos))
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
    (send *irtviewer* :draw-objects))
  )

(defun re-rotate-valve ()
  (if (eq *exit-flag* t) (return-from re-rotate-valve nil))
  (let ((phase 0))
    (if (< *exit-once* 1)
	(progn (go? `(switch-arms) nil)
	       (go? `(load-pose *arm*) nil)))
    (block :command-line
	   (dotimes (i 1000)
	     (if (eq *exit-flag* t) (return-from re-rotate-valve nil))
	     (case phase
		   (0 (print "adjust arm pose if needed and close hands using [magic-close] or nil for next step"))
		   (1 (print "rotate-valve or nil for next step"))
		   )
	     (setq ui (read-line))
	     (if (equal ui "exit-once")
		 (progn (setq *exit-once* 1)
			(return-from re-rotate-valve)))
	     (if (equal ui "nil")
		 (progn (case phase
			      (0 (warn "press enter~%"))
			      (t (return-from :command-line))
			      )
			(incf phase)))
	     (case phase
		   (1 (go? (read-from-string ui) nil))
		   (t (go? (read-from-string ui)))
		   )))
    (go? `(release-arm *arm*) nil)
    ))
  
;; debug

(defun for-test ()
  (go? `(send *aero* :hand :arms :open-pose))
  (go? `(send *aero* :hand-yaw-world *arm* -45))
  (go? `(send *aero* :upper :reach-arm *arm* :toward #f(0 0 1) :adjust #f(0 -1 0) :stop 400))
  (go? `(send *aero* :upper :reach-arm *arm* :toward #f(0 -1 0) :adjust #f(1 0 0) :stop 200))
  (go? `(send *aero* :upper *arm* :move-end-pos #f(80 0 0) :world))
  (go? `(send *aero* :upper *arm* :move-end-pos #f(0 -20 20) :world))
  (go? `(send *aero* :upper *arm* :move-end-pos #f(60 0 0) :world))
  (go? `(save-pose *arm*) nil)
  (go? `(send *aero* :magic-close *arm*))
  (go? `(rotate-valve *arm* 45 45) nil)
  (go? `(release-arm *arm*))
  (go? `(copy-arm-pose *arm* *narm*))
  (go? `(switch-arms) nil)
  (go? `(send *aero* :upper *arm* :move-end-pos (float-vector 0 (* *hand-free-space* 2) 0) :local))
  (go? `(save-pose *arm*) nil)
  (go? `(send *aero* :magic-close *arm*))
  (go? `(rotate-valve *arm* -90 45))
  (go? `(release-arm *arm*))
  (setq *exit-once* 0)
  (go? `(re-rotate-valve) nil)
  )


;; model samples

(defun sample-v0.02 ()
  (let (ui (phase 0))
    (cond ((< *exit-once* 1)
	   (go? `(send *aero* :hand :arms :open-pose))
	   ))
    (cond ((< *exit-once* 2)
	   (block :command-line-1
		  (dotimes (i 1000)
		    (if (eq *exit-flag* t) (return-from sample-v0.02 nil))
		    (case phase
			  (0 (print "rotate hand to desired grasp pose using [yaw-hand-world] or nil for next step"))
			  (1 (print "move arm to sweet position (don't close hand!) or nil for next step"))
			  (2 (print "rotate-valve or nil for next step"))
			  )
		    (setq ui (read-line))
		    (if (equal ui "exit-once")
			(progn (setq *exit-once* 1)
			       (return-from sample-v0.02)))
		    (if (equal ui "nil")
			(progn (case phase
				     (0 (warn "press enter~%"))
				     (1 (go? `(save-pose *arm*) nil)
					(go? `(send *aero* :magic-close *arm*)))
				     (t (return-from :command-line-1))
				     )
			       (incf phase)))
		    (case phase
			  (2 (go? (read-from-string ui) nil))
			  (t (go? (read-from-string ui)))
			  )))
	   ))
    (go? `(release-arm *arm*) nil)
    (go? `(copy-arm-pose *arm* *narm*))
    (go? `(switch-arms) nil)
    (setq phase 0)
    (block :command-line-2
	   (dotimes (i 1000)
	     (if (eq *exit-flag* t) (return-from sample-v0.02 nil))
	     (case phase
		   (0 (print "rotate other hand to desired grasp pose using [hand-yaw-world] or nil for next step"))
		   (1 (print "move other arm to sweet position (don't close hand!) or nil for next step"))
		   (2 (print "rotate-valve or nil for next step"))
		   )
	     (setq ui (read-line))
	     (if (equal ui "exit-once")
		 (progn (setq *exit-once* 2)
			(return-from sample-v0.02)))
	     (if (equal ui "nil")
		 (progn (case phase
			      (0 (warn "press enter~%"))
			      (1 (go? `(save-pose *arm*) nil)
				 (go? `(send *aero* :magic-close *arm*)))
			      (t (return-from :command-line-2))
			      )
			(incf phase)))
	     (case phase
		   (2 (go? (read-from-string ui) nil))
		   (t (go? (read-from-string ui)))
		   )))
    (go? `(release-arm *arm*) nil)
    (setq *exit-once* 0)
    (go? `(re-rotate-valve) nil)
    ))


;; real robot logs

