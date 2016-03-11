#!/usr/bin/env roseus

(ros::load-ros-manifest "aero_startup")
(ros::roseus "irex_demo")

(load "package://aeroeus/aero-interface.l")
(load-controllers)
(load "package://aeroeus/pre-release/inverse-kinematics-extension.l")

;; (ros::wait-for-service "/extract_object_features/perception_area")
(ros::wait-for-service "/point_cloud/perception_area")
(ros::wait-for-service "/color_memorizer/perception_area")
(ros::wait-for-service "/color_memorizer/find_match")

(defvar *timer* (instance mtimer :init))

(defvar *manip-area* (list '(-0.3 0.3) '(-0.3 0.3) '(0.0 1.0)) ) ;; x y z
(defvar *samples* (list (list "green" '(46 97) '(42 194) '(106 183)) ;; h s i
			(list "orange" '(-10 21) '(100 255) '(83 255))
			(list "blue" '(-128 -107) '(35 110) '(120 255))
			))

(defvar *null-area* (list '(0.0 0.0) '(0.0 0.0) '(0.0 0.0)))
(defvar *null-color* (list "null" '(0 0) '(0 0) '(0 0)))

(load "motions.l")
(load "nav.l")
(load "recog.l")


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun demo ()
  (let (color-tag color
	search-area found-object m search-flag
	go-went (go-sweet nil) y-direction
	ik-err grasp-pose grasp-status
	)
    ;; initate demo
    (start-pose 5000)
    (find-object *null-color* *null-area*)
    ;; set object to search
    (setq color-tag (set-object :timeout 60))
    (cond ((< color-tag 0)
	   (format t "failed distinguishing object")
	   (nod-no 500)
	   (return-from demo nil)
	   ) ;; failed distinguishing object
	  )
    (nod-yes 500)
    (setq color (elt *samples* color-tag))
    (format t "~A~%" (car color))
    ;; move base to face work station
    (cond ((eq *real* t)
	   (send *ri* :go-pos 0 0 110)
	   (unix::usleep (* 8000 1000))
	   ))
    ;; ready for manipulation
    (run-pose 500)
    ;; search for object
    (setq search-area *manip-area*)
    (setq found-object (find-object color search-area))
    (setq search-flag t)
    (while (and (< (cadr (elt search-area 2)) 5.0) ;; search area limit
		(not (> found-object 0))
		search-flag) ;; object not found
      (setq search-area (list (list -1.0 1.0)
			      (list -1.0 1.0)
			      (list 0.0 (+ (cadr (elt search-area 2)) 0.2))
			      )) ;; expand search-area
      (setq found-object (find-object color search-area))
      ;; re-initiate *tl* so that it does not find ghost tfs
      (setf *tl* (instance ros::transform-listener :init))
      ;; maybe tf was bad timing
      (unix::usleep (* 1000 1000))
      (setq m (send *tl* :lookup-transform "leg_base_link" "object_l" (ros::time)))
      (if (not (eq m nil))
	  (if (and (not (eps= (elt (send m :pos) 0) 0 0.1))
		   (not (eps= (elt (send m :pos) 1) 0 0.1))
		   (not (eps= (elt (send m :pos) 2) 0 0.1))
		   (not (< (elt (send m :pos) 2) -500)) ;; stash floor
		   (not (< (elt (send m :pos) 0) 1000)) ;; too close maybe noise
		   )
	      (setq search-flag nil)
	    )
	) ;; if the tf was published even once, then if would be t
      )
    (cond ((>= (cadr (elt search-area 2)) 5.0)
	   (format t "failed finding actual object")
	   (return-from demo (car color))
	   ) ;; failed finding actual object
	  )
    ;; move base forward till robot is near object
    (setq go-went (go-for-target '(100000 0) '(0.72 1000)))
    (cond ((< (car go-went) 0)
	   (format t "failed moving toward object")
	   (return-from demo go-went)
	   ) ;; failed moving toward object
	  )
    ;; get to sweet manip position
    (setq m (send *tl* :lookup-transform "leg_base_link" "object_l" (ros::time)))
    (cond ((or (< (elt (send m :pos) 1) 0)
	       (> (elt (send m :pos) 1) 180))
	   (cond ((< (elt (send m :pos) 1) 0) ;; object is on far right
		  (setq y-direction "go-right")
		  (setq go-sweet (go-for-target '(0 -1000) '(1000 0.35)))
		  )
		 ((> (elt (send m :pos) 1) 180) ;; object is on far left
		  (setq y-direction "go-left")
		  (setq go-sweet (go-for-target '(0 1000) '(1000 0.18)))
		  )
		 )
	   (cond ((< (car go-sweet) 0)
		  (format t "failed adjusting manip position")
		  (return-from demo go-sweet)
		  ) ;; failed adjusting position
		 )
	   ))
    ;; manipulate at work station
    (unix::usleep (* 3000 1000)) ;; for some reason move-waist is not called
    (cond ((eq *real* t)
	   (send *ri* :move-waist (float-vector 0 0 -200) :world)
	   (unix::usleep (* 2200 1000))
	   ))
    (find-object color *manip-area*)
    (unix::usleep (* 2000 1000)) ;; has to wait for stable configure, 5000 for bad network
    ;; solve ik
    (setq ik-err (send *aero* :grasp-object :larm :torso t :revert-if-fail nil :offset-z 50))
    (setq grasp-pose (send *aero* :angle-vector)) ;; save pose
    (setq waist-z (elt (cadr ik-err) 2)) ;; save height
    (cond ((eq *real* t)
	   (send *ri* :ungrasp :larm)
	   (unix::usleep (* 1000 1000))
	   (send *ri* :move-waist (cadr ik-err))
	   (unix::usleep (* 1000 1000))
	   (manip-pose 1000)
	   (setq move-time 3000)
	   (send *aero* :angle-vector grasp-pose)
	   (cond ((not (eq (send *aero* :larm :move-end-pos #f(-50 80 0)
				 :local :rotation-axis nil :traslation-axis :z)
			   nil))
		  (send *ri* :angle-vector (send *aero* :angle-vector) 2000)
		  (unix::usleep (* 2000 1000))
		  (send *aero* :angle-vector grasp-pose)
		  (setq move-time 1000)
		  ))
	   (send *ri* :angle-vector (send *aero* :angle-vector) move-time)
	   (unix::usleep (* move-time 1000))
	   (setq grasp-status (send *ri* :grasp :larm))
	   (unix::usleep (* 1000 1000))
	   ))
    (back-pose 5000)
    ;; move back from sweet position
    (if (not (eq go-sweet nil))
	(if (equal y-direction "go-left") ;; moved direction
	    (go-for-time 0 -1000 0 (cadr go-sweet))
	  (go-for-time 0 1000 0 (cadr go-sweet))
	  )
      )
    ;; move base to face original station
    (cond ((eq *real* t)
	   (send *ri* :go-pos 0 0 -110)
	   (unix::usleep (* 8000 1000))
	   ))
    ;; move base backward to starting point
    (go-for-time 100000 0 0 (+ (cadr go-went) 0.4))
    ;; grasp failed
    (cond ((equal grasp-status "grasp failed")
	   (end-pose 500)
	   (sorry 1000)
	   (start-pose 3000)
	   (return-from demo nil)
	   ))
    ;; place object
    (unix::usleep (* 3000 1000)) ;; for some reason move-waist is not called
    (cond ((eq *real* t)
	   (send *ri* :move-waist (v+ #f(0 0 -200) (cadr ik-err)) :world)
	   (unix::usleep (* 2200 1000))
	   ))
    (send *aero* :angle-vector grasp-pose)
    (send *aero* :head :neck-p :joint-angle 0)
    (send *aero* :head :neck-r :joint-angle 0)
    (send *aero* :head :neck-y :joint-angle 0)
    (cond ((eq *real* t)
	   (send *ri* :angle-vector (send *aero* :angle-vector) 3000)
	   (unix::usleep (* 3000 1000))
	   (send *ri* :ungrasp :larm)
	   (unix::usleep (* 1000 1000))
	   ))
    (end-pose 1000)
    ))

(aero-init)
(objects *aero*) ;; this has to be called or error occurs
(setq *real* t)
