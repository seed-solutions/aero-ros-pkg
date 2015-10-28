#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(setq *tfb* (instance ros::transform-broadcaster :init))
(setq *frame-id* "leg_base_link")

(setq *tl* (instance ros::transform-listener :init))

(defun tf (x y z)
  (let ((cds (make-coords :pos (float-vector x y z)))
	)
    (send *tfb* :send-transform cds
	  *frame-id*
	  "test_tf"
	  (ros::time-now))
  ))

(defun ~tf (&optional (arm nil))
  (let (m
	z
	target-p
	target-m
	)
    (ros::spin-once)
    (setq m (send *tl* :lookup-transform "leg_base_link" "object" (ros::time)))
    (if (eq m nil) (return-from ~tf nil))

    (setq target-p (v+ (send m :pos)
		       (send (send (car (send *aero* :links))
				   :copy-worldcoords) :pos)))

    (setq target-m (m* (send m :rot)
		       (rotation-matrix (/ pi 2) #f(0 1 0))
		       (inverse-matrix (send m :rot))))
    (setq z (transform target-m #f(0 0 1)))
    (cond ((eq arm :larm)
	   (if (< (v. z #f(-1 -1 1)) (v. z #f(1 1 -1)) )
	       (setq target-m (m* target-m
				  (rotation-matrix pi #f(1 0 0))
				  (inverse-matrix target-m)))
	     )
	   )
	  ((eq arm :rarm)
	   (if (< (v. z #f(-1 1 1)) (v. z #f(1 -1 -1)) )
	       (setq target-m (m* target-m
				  (rotation-matrix pi #f(1 0 0))
				  (inverse-matrix target-m)))
	     )
	   )
	  )

    (setq m (make-coords :pos target-p :rot target-m))
    (send m :draw-on :flush t)
    m))
