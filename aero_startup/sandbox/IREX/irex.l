#!/usr/bin/env roseus

(ros::load-ros-manifest "aero_startup")
(ros::roseus "irex_demo")

(load "package://aeroeus/aero-interface.l")
(load-controllers)
(load "package://aeroeus/pre-release/inverse-kinematics-extension.l")

(ros::wait-for-service "/extract_object_features/perception_area")
(ros::wait-for-service "/color_memorizer/perception_area")
(ros::wait-for-service "/color_memorizer/find_match")

(defvar *timer* (instance mtimer :init))

(defvar *manip-area* (list '(-0.3 0.3) '(-0.3 0.3) '(0.0 1.0)) ) ;; x y z
(defvar *samples* (list (list "green" '(46 97) '(42 194) '(106 183)) ;; h s i
			(list "orange" '(2 17) '(252 255) '(67 246))
			(list "red" '(-14 2) '(160 255) '(45 76))
			))

(defvar *null-area* (list '(0.0 0.0) '(0.0 0.0) '(0.0 0.0)))
(defvar *null-color* (list "null" '(0 0) '(0 0) '(0 0)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun start-pose (&optional (time 3000))
  (send *aero* :reset-manip-pose)
  (cond ((eq *real* t)
	 (send *ri* :move-waist #f(0 0 0) :world)
	 (unix::usleep (* 2200 1000))
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun run-pose (&optional (time 3000))
  (send *aero* :reset-manip-pose)
  (send *aero* :head :neck-p :joint-angle 40)
  (cond ((eq *real* t)
	 (send *ri* :move-waist #f(0 0 0) :world)
	 (unix::usleep (* 2200 1000))
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun manip-pose (&optional (time 5000))
  (send *aero* :angle-vector
	#f(0.0 -15.0 0.0 -90.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 15.0 0.0 -90.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 40.0 0.0))
  (cond ((eq *real* t)
	 (send *ri* :move-waist #f(0 0 -200) :world)
	 (unix::usleep (* 2200 1000))
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun set-object (&key (timeout 60))
  (let (req res
	(h_max_v nil) (h_min_v nil)
	(s_max_v nil) (s_min_v nil)
	(i_max_v nil) (i_min_v nil)
	)
    (setq req (instance aero_startup::PointHSIArrayRequest :init))
    (send req :size (length *samples*))
    (dolist (x *samples*)
      (push (car (elt x 1)) h_min_v) (push (cadr (elt x 1)) h_max_v)
      (push (car (elt x 2)) s_min_v) (push (cadr (elt x 2)) s_max_v)
      (push (car (elt x 3)) i_min_v) (push (cadr (elt x 3)) i_max_v)
      )
    (nreverse h_min_v) (nreverse h_max_v)
    (nreverse s_min_v) (nreverse s_max_v)
    (nreverse i_min_v) (nreverse i_max_v)
    (send req :h_cap_v h_max_v) (send req :h_v h_min_v)
    (send req :s_cap_v s_max_v) (send req :s_v s_min_v)
    (send req :i_cap_v i_max_v) (send req :i_v i_min_v)
    (send req :time timeout)
    (setq res (ros::service-call "/color_memorizer/find_match" req))
    (send res :status) ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun find-object (hsi xyz &optional (precise t))
  (let (req res
	)
    (setq req (instance aero_startup::PointXYZHSIRequest :init))
    (send req :x (car (elt xyz 0))) (send req :x_cap (cadr (elt xyz 0)))
    (send req :y (car (elt xyz 1))) (send req :y_cap (cadr (elt xyz 1)))
    (send req :z (car (elt xyz 2))) (send req :z_cap (cadr (elt xyz 2)))
    (send req :h (car (elt hsi 1))) (send req :h_cap (cadr (elt hsi 1)))
    (send req :s (car (elt hsi 2))) (send req :s_cap (cadr (elt hsi 2)))
    (send req :i (car (elt hsi 3))) (send req :i_cap (cadr (elt hsi 3)))
    (send req :precise precise)
    (setq res (ros::service-call "/extract_object_features/perception_area" req))
    (send res :status) ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun go-for-target (xy ~xy)
  (let (req res
	)
    (setq req (instance aero_startup::ObjectGoXYZHSIRequest :init))
    (send req :object "object_l")
    (send req :go_x (elt xy 0)) ;; any large number if x movement
    (send req :go_y (elt xy 1)) ;; any large number if y movement
    (send req :until_x (elt ~xy 0)) ;; stop distance, large number for no limits
    (send req :until_y (elt ~xy 1)) ;; stop distance, large number for no limits
    (setq res (ros::service-call "/go_run/target_object" req))
    (list (send res :status) (send res :time)) ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun go-for-time (x y theta time)
  (let (req res
	)
    (setq req (instance aero_startup::GoTimeRequest :init))
    (send req :go_x x)
    (send req :go_y y)
    (send req :go_theta theta)
    (send req :time time)
    (setq res (ros::service-call "/go_run/target_time" req))
    (send res :status) ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun demo ()
  (let (color-tag color
	search-area found-object m search-flag
	go-went
	ik-err grasp-pose
	)
    ;; initate demo
    (start-pose 5000)
    (setq search-area *null-color* *null-area*)
    ;; set object to search
    (setq color-tag (set-object :timeout 60))
    (cond ((< color-tag 0)
	   (format t "failed distinguishing object")
	   (return-from demo nil)
	   ) ;; failed distinguishing object
	  )
    (setq color (elt *samples* color-tag))
    (format t "~A~%" (car color))
    ;; ready for manipulation
    (run-pose 1000)
    ;; move base to face work station
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
		   (not (eps= (elt (send m :pos) 2) 0 0.1)))
	      (setq search-flag nil)
	    )
	) ;; if the tf was published even once, then if would be t
      )
    (cond ((>= (cadr (elt search-area 2)) 5.0)
	   (format t "failed finding actual object")
	   (return-from demo nil)
	   ) ;; failed finding actual object
	  )
    ;; move base forward till robot is near object
    (setq go-went (go-for-target '(100000 0) '(0.8 1000)))
    (cond ((< (car go-went) 0)
	   (format t "failed moving toward object")
	   (return-from demo nil)
	   ) ;; failed moving toward object
	  )
    ;; manipulate at work station
    (manip-pose 1000)
    (find-object color *manip-area*)
    ;; solve ik
    (setq ik-err (send *aero* :grasp-object :larm :torso t :revert-if-fail nil))
    (setq grasp-pose (send *aero* :angle-vector)) ;; save pose
    (cond ((eq *real* t)
	   (send *ri* :ungrasp :larm)
	   (unix::usleep (* 1000 1000))
	   (send *ri* :move-waist (cadr ik-err))
	   (unix::usleep (* 1000 1000))
	   (send *ri* :angle-vector (send *aero* :angle-vector) 3000)
	   (unix::usleep (* 3000 1000))
	   (send *ri* :grasp :larm)
	   (unix::usleep (* 1000 1000))
	   ))
    (run-pose 2000)
    ;; move base backward to starting point
    (go-for-time -100000 0 0 (cadr go-went))
    ;; move base to face original station
    ;; place object
    (manip-pose 1000)
    (send *aero* :angle-vector grasp-pose)
    (cond ((eq *real* t)
	   (send *ri* :angle-vector (send *aero* :angle-vector) 5000)
	   (unix::usleep (* 5000 1000))
	   (send *aero* :ungrasp :larm)
	   (unix::usleep (* 1000 1000))
	   ))
    (start-pose 5000)
    ))

(aero-init)
(objects *aero*)
