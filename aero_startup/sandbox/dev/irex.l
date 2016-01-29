#!/usr/bin/env roseus

(ros::load-ros-manifest "aero_startup")
(ros::roseus "irex_demo")

(load "package://aeroeus/aero-interface.l")
(load-controllers)
(load "package://aeroeus/pre-release/inverse-kinematics-extension.l")

(defvar *timer* (instance mtimer :init))

(defvar *manip-area* (list '(-0.3 0.3) '(-0.3 0.3) '(0.0 1.0)) ) ;; x y z
(defvar *samples* (list
		   (list "red" '(-20 4) '(254 255) '(40 116))
		   (list "green" '(46 97) '(42 194) '(106 183)) ;; h s i
		   (list "orange" '(-10 21) '(100 255) '(83 255))
		   (list "blue" '(-128 -107) '(35 110) '(120 255))
		   ))

(defvar *null-area* (list '(0.0 0.0) '(0.0 0.0) '(0.0 0.0)))
(defvar *null-color* (list "null" '(0 0) '(0 0) '(0 0)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun start-pose (&optional (time 3000))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 0.0 0.0))
  (cond ((eq *real* t)
	 (send *ri* :move-waist #f(0 0 0) :world)
	 (unix::usleep (* 2200 1000))
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun def-pose (&optional (z 0.0)) ;; todo
  (let (time)
    (send *ri* :ungrasp :arms)
    (unix::usleep (* 1000 1000))
    (setq time 2000)
    (setq time (+ time
		  (* (abs (send *aero* :torso :waist-p :joint-angle)) 50)))
    (send *aero* :angle-vector
	  #f(-13.9145 0.0 0.0 -133.689 0.0 0.0 0.0 -50.0 0.0 0.0
	     0.0 -13.9145 0.0 0.0 -137.416 0.0 0.0 0.0 50.0 0.0 0.0 0.0
	     0.0 0.0 0.0 0.0 40.0 0.0))
    (cond ((eq *real* t)
	   (send *ri* :move-waist (float-vector 0 0 z) :world)
	   (unix::usleep (* 1000 1000))
	   (send *ri* :angle-vector (send *aero* :angle-vector) time)
	   (unix::usleep (* (floor time) 1000))
	   ))
    ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun object-grasp-test-once (x y &optional (f 0.4) (w -0.3))
  (let (ik-err grasp-pose move-time waist-pose)
    (setq ik-err (send *aero* :grasp-object :larm :torso t
		       :revert-if-fail nil :offset-x x :offset-z y))
    (setq grasp-pose (send *aero* :angle-vector))
    (cond ((eq *real* t)
	   (send *ri* :ungrasp :larm)
	   (unix::usleep (* 1000 1000))
	   (setq waist-pose (send *ri* :move-waist (cadr ik-err))) 
	   (if (or (equal (subseq waist-pose 0 3) "nan")
		   (equal (subseq waist-pose 0 7) "invalid"))
	       (return-from object-grasp-test-once
			    (list nil (car ik-err) (cadr ik-err)))
	     )
	   (unix::usleep (* 1000 1000))
	   (setq move-time 3000)
	   (cond ((not (eq (send *aero* :larm :move-end-pos
				 #f(-50 80 0) :local :rotation-axis
				 nil :traslation-axis :z)
			   nil))
		  (format t "pre-ik success ~%")
		  (send *ri* :angle-vector (send *aero* :angle-vector) 2000)
		  (setq move-time 1000)
		  (unix::usleep (* 2000 1000))
		  )
		 (t (format t "pre-ik failed ~%"))
		 )
	   (format t "move robot ~%")
	   (send *aero* :angle-vector grasp-pose)
	   (send *ri* :angle-vector (send *aero* :angle-vector) move-time)
	   (unix::usleep (* move-time 1000))
	   (setq grasp-status (send *ri* :grasp :larm :tfail f :twarn w))
	   ;; (unix::usleep (* 1000 1000))
	   ))
    (list (if (or (equal grasp-status "grasp failed")
		  (equal grasp-status "grasp bad")) nil t)
	  (car ik-err)
	  (cadr ik-err)
	  )
    ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun go-for-time (x y theta time) ;; sec
  (let (req res
	)
    (setq req (instance aero_startup::GoTimeRequest :init))
    (send req :go_x x)
    (send req :go_y y)
    (send req :go_theta theta)
    (send req :time time)
    (setq res (ros::service-call "/run_by_time/target_time" req))
    (send res :status) ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun refind-target (arm)
  (let (status)
    (setq status -1)
    (while (< status 0)
      (setq status (set-to-recog-mode))
      (unix::usleep (* 100 1000))
      )
    (unix::usleep (* 3000 1000))
    (setq status -1)
    (while (< status 0)
      (setq status (send (send *ri* :tf arm) :pos))
      (setq status (set-to-track-mode (format nil "~A ~A ~A world"
					      (elt status 0)
					      (elt status 1)
					      (elt status 2)) ))
      (unix::usleep (* 100 1000))
      )
    ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; (defun run (arm ~x ~z &key (dt 0.5) (dz -50.0))
(defun run (arm ~z &key (dz -50.0))
  (let ((z 0.0)
	(+x 0.0)
	(+z 0.0)
	test-result
	(test-results nil)
	(status -1)
	)
    (refind-target arm)
    ;; (dotimes (i ~x)
      (dotimes (j ~z)
	;; reset pose
	(def-pose z)
	;; refind target if target is lost
	(unix::usleep (* 1000 1000))
	(if (eq (send *ri* :tf arm) nil) (refind-target arm))
	(setq +z 100.0) ;; maximum prior offset
	(if (> (+ z +z) 0) (setq +z 0.0)) ;; offset cannot exceed +0.0
	(setq test-result (object-grasp-test-once 0.0 +z)) ;; try grasp
	;; save result
	(setq test-result (append test-result
				  (list +x z +z)
				  (list (send *ri* :tf arm))))
	(push test-result test-results)
	(cond ((eq (car test-result) nil) ;; when grasp fails
	       (def-pose z) ;; reset pose
	       (block get-magic-number
		      (while 1 ;; escapes when grasp succeeds or z becomes too low
			;; reset waist position to z
			(send *ri* :move-waist (float-vector 0 0 z) :world)
			(setq +z (- +z 50.0)) ;; new z position
			(if (< +z -100.0) (return-from get-magic-number))
			;; find possible x range for this z position
			(let (wz waist-calc (while-t t) (jj-times 0))
			  (setq wz (+ z +z))
			  (setq +x 0.0)
			  (while while-t ;; escapes when x is not calculable
			    (setq +x (+ +x 50.0)) ;; new x position
			    (send *ri* :move-waist (float-vector +x 0 wz) :world)
			    (unix::usleep (* 1000 1000))
			    ;; get current waist position
			    (setq waist-calc (send *ri* :move-waist #f(0 0 0)))
			    ;; when actual position does not match desired
			    (if (not
				 (and (eps= (- (elt waist-calc 0) +x) 0.0)
				      (eps= (- (elt waist-calc 1) wz) 0.0)))
				(setq while-t nil) ;; escape from while
			      )
			    (incf jj-times)
			    )
			  (setq +x 0.0)
			  (dotimes (jj jj-times) ;; try for all x-range
			    ;; reset pose
			    (def-pose z)
			    ;; refind target if target is lost
			    (unix::usleep (* 1000 1000))
			    (if (eq (send *ri* :tf arm) nil) (refind-target arm))
			    ;; try grasp
			    (setq test-result (object-grasp-test-once +x +z))
			    ;; save result
			    (setq test-result (append test-result
						      (list +x z +z)
						      (list (send *ri* :tf arm))))
			    (push test-result test-results)
			    (if (eq (car test-result) t) ;; grasp succeeds
				(return-from get-magic-number)
			      )
			    (setq +x (+ +x 10.0)) ;; try next x
			    )) ;; let
			(def-pose z)
			) ;; while
		      ) ;; block
	       ))
	(setq z (+ z dz)) ;; next z position
	) ;; dotimes j
      (def-pose)
      ;; (go-for-time -100 0 0 dt) ;; move a little bit far from current position
      ;; (setq z 0.0) ;; reset z
      ;; ) ;; dotimes i
    test-results))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun save (name data)
  (let (i)
    (with-open-file (str (concatenate string name "_raw.txt") :direction :output
			 :if-exists :supersede)
		    (setq i 0)
		    (dolist (x data)
		      (format str "data ~A:~%  result: ~A~%  goal: ~A~%  diff: ~A~%  off-x: ~A~%  off-z0: ~A~%  off-z1: ~A~%  end: ~A~%"
			      i (elt x 0) (elt x 1) (elt x 2)
			      (elt x 3) (elt x 4) (elt x 5) (elt x 6))
		      (incf i)
		      )
		    )
    ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun ready? (direction)
  (cond ((eq direction :cw)
	 (if (> (send *aero* :torso :waist-y :joint-angle) 80)
	     (return-from ready? nil)
	   )
	 (if (< (send *aero* :torso :waist-y :joint-angle) -80)
	     (send *aero* :torso :waist-y :joint-angle 0)
	   (send *aero* :torso :waist-y :joint-angle 90)
	   )
	 (send *ri* :go-pos 0 0 -48)
	 (send *ri* :angle-vector (send *aero* :angle-vector) 5000)
	 )
	((eq direction :ccw)
	 (if (< (send *aero* :torso :waist-y :joint-angle) -80)
	     (return-from ready? nil)
	   )
	 (if (> (send *aero* :torso :waist-y :joint-angle) 80)
	     (send *aero* :torso :waist-y :joint-angle 0)
	   (send *aero* :torso :waist-y :joint-angle -90)
	   )
	 (send *ri* :go-pos 0 0 48)
	 (send *ri* :angle-vector (send *aero* :angle-vector) 5000)
	 )
	(t nil)
	)
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun sorry (&optional (time 1000))
  (let ((a-pose #f(0.0 -2.0 0.0 -30.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
		   0.0 0.0 0.0 -30.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
		   0.0 30.0 0.0 0.0 40.0 0.0))
	(b-pose #f(-0.035975 -2.05717 0.596428 -26.5836 -0.26772 6.61294 -0.036075 0.0 0.0 0.0 0.0
		   -0.038505 0.005706 -0.022469 -26.8227 0.026125 6.86112 0.006531 0.0 0.0 0.0 0.0
		   0.0 20.0 0.0 0.0 40.0 0.0))
	)
    (send *aero* :angle-vector a-pose)
    (cond ((eq *real* t)
	   (send *ri* :angle-vector (send *aero* :angle-vector) time)
	   (unix::usleep (* time 1000))
	   ))
    (send *aero* :angle-vector b-pose)
    (cond ((eq *real* t)
	   (send *ri* :angle-vector (send *aero* :angle-vector) (* time 0.5))
	   (unix::usleep (* time 500))
	   ))
    (send *aero* :angle-vector a-pose)
    (cond ((eq *real* t)
	   (send *ri* :angle-vector (send *aero* :angle-vector) (* time 0.5))
	   (unix::usleep (* time 500))
	   ))
    (send *aero* :angle-vector b-pose)
    (cond ((eq *real* t)
	   (send *ri* :angle-vector (send *aero* :angle-vector) (* time 0.5))
	   (unix::usleep (* time 500))
	   ))
    (send *aero* :angle-vector a-pose)
    (cond ((eq *real* t)
	   (send *ri* :angle-vector (send *aero* :angle-vector) (* time 0.5))
	   (unix::usleep (* time 500))
	   ))
    ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun run-pose (&optional (time 2000))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 40.0 0.0))
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun manip-pose (&optional (time 1000))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 120.0 -30.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 40.0 0.0))
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) 500)
	 (unix::usleep (* 500 1000))
	 ))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 20.0 100.0 -80.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 40.0 0.0))
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 20.0 0.0 -100.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 40.0 0.0))
  (cond ((eq *real* t)
         (send *ri* :angle-vector (send *aero* :angle-vector) time)
         (unix::usleep (* time 1000))
         ))
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun back-pose (&optional (time 2000))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 20.0 0.0 -100.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 40.0 0.0))
  (cond ((eq *real* t)
	 (send *ri* :move-waist #f(0 0 0) :world)
	 (unix::usleep (* 2200 1000))
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun end-pose (&optional (time 1000))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 20.0 0.0 -100.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 0.0 0.0))
  (cond ((eq *real* t)
	 (send *ri* :move-waist #f(0 0 0) :world)
	 (unix::usleep (* 2200 1000))
	 (send *ri* :angle-vector (send *aero* :angle-vector) 5000)
	 (unix::usleep (* 5000 1000))
	 ))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 20.0 100.0 -80.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 0.0 0.0))
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 120.0 -30.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 0.0 0.0))
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 0.0 0.0))
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) 500)
	 (unix::usleep (* 500 1000))
	 ))
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun nod-no (&optional (time 500))
  (send *aero* :head :neck-y :joint-angle -30)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  (send *aero* :head :neck-y :joint-angle 30)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) (* time 2))
	 (unix::usleep (* time 2000))
	 ))
  (send *aero* :head :neck-y :joint-angle 0)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun nod-yes (&optional (time 500))
  (send *aero* :head :neck-p :joint-angle 40)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  (send *aero* :head :neck-p :joint-angle 0)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  (send *aero* :head :neck-p :joint-angle 40)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  (send *aero* :head :neck-p :joint-angle 0)
  (cond ((eq *real* t)
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
    (setq res (ros::service-call "/point_cloud/perception_area" req))
    (send res :status) ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun go-for-target (xy ~xy &optional (time_out 5000))
  (let (req res
	)
    (setq req (instance aero_startup::TimeFromGoXYRequest :init))
    (send req :object "object_l")
    (send req :go_x (elt xy 0)) ;; any large number if x movement
    (send req :go_y (elt xy 1)) ;; any large number if y movement
    (send req :until_x (elt ~xy 0)) ;; stop distance, large number for no limits
    (send req :until_y (elt ~xy 1)) ;; stop distance, large number for no limits
    (send req :time_out_ms time_out)
    (setq res (ros::service-call "/run_by_vision/target_object" req))
    (list (send res :status) (send res :time)) ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun reconfigure (xyz str)
  (let (req res
        )
    (setq req (instance aero_startup::PointXYZRequest :init))
    (send req :x (car (elt xyz 0))) (send req :x_cap (cadr (elt xyz 0)))
    (send req :y (car (elt xyz 1))) (send req :y_cap (cadr (elt xyz 1)))
    (send req :z (car (elt xyz 2))) (send req :z_cap (cadr (elt xyz 2)))
    (setq res (ros::service-call str req))
    (send res :status) ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun set-to-track-mode (msg)
  (let (req res
	)
    (setq req (instance aero_startup::ProcessSleepRequest :init))
    (send req :sleep 1)
    (setq res (ros::service-call "/plane_detection/sleep" req))
    (if (< (send res :status) 0)
	(return-from set-to-track-mode
		     (format t "error: ~A~%" (send res :status)))
      )
    (send req :sleep 0)
    (send req :message msg)
    (setq res (ros::service-call "/object_tracker/sleep" req))
    (send res :status) ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun set-to-recog-mode ()
  (let (req res
	)
    (setq req (instance aero_startup::ProcessSleepRequest :init))
    (send req :sleep 1)
    (setq res (ros::service-call "/object_tracker/sleep" req))
    (if (< (send res :status) 0)
	(return-from set-to-recog-mode
		     (format t "error: ~A~%" (send res :status)))
      )
    (send req :sleep 0)
    (setq res (ros::service-call "/plane_detection/sleep" req))
    (send res :status) ))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun demo ()
  (let (color-tag color
	search-area m search-flag
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
    (find-object color search-area)
    (setq search-flag t)
    (while (and (< (cadr (elt search-area 2)) 5.0) ;; search area limit
		search-flag) ;; object not found
      (setq search-area (list (list -1.0 1.0)
			      (list -1.0 1.0)
			      (list 0.0 (+ (cadr (elt search-area 2)) 0.2))
			      )) ;; expand search-area
      (find-object color search-area)
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
;; (objects *aero*)
(setq *real* t)

;; (setq ((send *aero* :torso :waist-y) . min-angle) -90)
;; (setq ((send *aero* :torso :waist-y) . max-angle) 90)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun search-debug ()
  (let (color search-area m search-flag
	)
    (setq color (elt *samples* 0))
    (setq search-area *manip-area*)
    (find-object color search-area)
    (setq search-flag t)
    (while (and (< (cadr (elt search-area 2)) 5.0) ;; search area limit
		search-flag) ;; object not found
      (setq search-area (list (list -1.0 1.0)
			      (list -1.0 1.0)
			      (list 0.0 (+ (cadr (elt search-area 2)) 0.2))
			      )) ;; expand search-area
      (find-object color search-area)
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
	   (return-from search-debug (car color))
	   ) ;; failed finding actual object
	  )
    ))


;;;;;;;;;;;;;;;;;;;;;;;;
(defun demo2 (&key (move-x 0.7) (offset-z 50) (offset-y 0.0) (offset-x 20.0))
  (def-pose)
  (refind-target :larm)
  (go-for-target #f(100 0) (float-vector move-x 1000))
  (setq ik-err (send *aero* :grasp-object :larm :torso t :revert-if-fail nil
		     :offset-z offset-z))
  (print ik-err)

  (setq ik-err-x (elt (cadr ik-err) 0)
	ik-err-y (elt (cadr ik-err) 1)
	ik-err-z (elt (cadr ik-err) 2))
  (setq move-waist-x ik-err-x
	move-waist-y ik-err-y
	move-waist-z ik-err-z
	;; move-hand-x 0.0
	move-hand-x offset-x
	move-hand-y (+ offset-y ik-err-y)
	move-hand-z 0.0)

  (when (> ik-err-z 0)
    (setq move-waist-z (- ik-err-z)
	  move-hand-z (* ik-err-z 2.0)))
  (setq move-waist-vec (float-vector move-waist-x move-waist-y move-waist-z)
	move-hand-vec (float-vector move-hand-x move-hand-y move-hand-z))
  (send *aero* :larm :move-end-pos move-hand-vec :world :rotation-axis nil)

  (setq grasp-pose (send *aero* :angle-vector))
  (send *ri* :ungrasp :larm)
  (unix::usleep (* 1000 1000))
  (send *ri* :move-waist move-waist-vec)
  (unix::usleep (* 1000 1000))
  (send *aero* :angle-vector grasp-pose)
  (cond ((not (eq (send *aero* :larm :move-end-pos #f(-50 80 0)
                        :local :rotation-axis nil :traslation-axis :z) nil))
         (send *ri* :angle-vector (send *aero* :angle-vector) 2000)
         (unix::usleep (* 2000 1000))
         (send *aero* :angle-vector grasp-pose)
         (setq move-time 1000)
         ))
  (send *ri* :angle-vector grasp-pose)
  (unix::usleep (* move-time 1000))
  (setq grasp-status (send *ri* :grasp :larm))
  (unix::usleep (* 1000 1000))
  (cond ((equal grasp-status "grasp failed")
         (end-pose 500)
         (sorry 1000)
	 (def-pose 3000)
         )
	(t
	 (send *aero* :reset-manip-pose)
	 (send *ri* :angle-vector (send *aero* :angle-vector) 3000)
	 (nod-yes)
	 )
	)
  )
