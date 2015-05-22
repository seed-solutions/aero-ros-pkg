#!/usr/bin/env roseus
(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
;; (require :aero-upper "package://aero_ros_bridge/models/aero_upper.l")

(ros::load-ros-manifest "trajectory_msgs")
(ros::load-ros-manifest "pr2_controllers_msgs")

(defmethod AeroUpperRobot-robot
  (:hand (&rest args)
     (if (null args) (return-from :hand nil))
     (let ((key (car args))
           (nargs (cdr args)))
       (unless (keywordp key)
         (return-from :hand nil))
       (case key
	 (:arms (progn (send self :hand-poses :larm nargs)
		       (send self :hand-poses :rarm nargs)))
	 (:larm (send self :hand-poses :larm nargs))
	 (:rarm (send self :hand-poses :rarm nargs))
	 (t nil)
	 )
       )
     )
  (:hand-poses (hand args)
    (let ((pose (car args))
	  (angle 0)
	  (hand-vector (float-vector
			(send self hand :t-1p :joint-angle)
			(send self hand :f-1p :joint-angle)
			0 0))
	  )
      (case pose
	    (:reset-pose (setq hand-vector #f(0 0 0 0)))
	    (:open-pose (if (eq hand :larm) (setq hand-vector #f(-50 50 0 0))
			  (setq hand-vector #f(50 -50 0 0))))
	    (:close-pose (if (eq hand :larm) (setq hand-vector #f(50 -50 0 0))
			   (setq hand-vector #f(-50 50 0 0))))
	    (:joint-angle (progn
			    (setq angle (car (cdr args)))
			    (if (eq hand :larm)
				(setq hand-vector
				      (float-vector angle (- angle) 0 0))
			      (setq hand-vector
				    (float-vector (- angle) angle 0 0))
			      )
			    ))
	    (t (return-from :hand-poses nil))
	    )
      (send self hand :t-1p :joint-angle (elt hand-vector 0))
      (send self hand :f-1p :joint-angle (elt hand-vector 1))
      (send self hand :f-2p :joint-angle (elt hand-vector 2))
      (send self hand :f-3p :joint-angle (elt hand-vector 3))
      hand-vector)
    )
  ;; (:fix-upper-to-coords (&rest coords)
  ;;    (if (null coords)
  ;; 	 (if (boundp '*aero-lower*)
  ;; 	     (send self :newcoords
  ;; 		   (send (send *aero-lower* :torso :end-coords)
  ;; 			 :worldcoords))
  ;; 	   )
  ;;      (send self :newcoords (car coords))
  ;;      )
  ;;    )
  (:move-safe (arm to &key (bits 10)
			   (world nil))
     (let* ((delta-to 0)
	    (coords :local)
	    (step-before-angle-vector)
	    )
       (if (eq world t) (setq coords :world))
       (setq delta-to (scale (/ 1.0 bits) to))
       (dotimes (i bits)
	 (setq step-before-angle-vector (send self :angle-vector))
	 (send self arm :move-end-pos delta-to coords) ; move a step
	 (cond ((not (send self :collision-check-all)) ; if collides
		;; (send self arm :move-end-pos (scale (- 1) delta-to) coords) 
		(send self :angle-vector step-before-angle-vector) ; go back a step
		(return-from :move-safe nil)
		))
	 ))
     t)
  (:reach-arm (arm &key (toward (float-vector 1 0 0))
		        (adjust (float-vector 0 0 1))
			(bits 1)
			(stop 10))
     (let* ((flag nil))
       (dotimes (j stop)
	 ; if arm can nolonger be stretched finish                         
	 (if (eq flag t) (return))
	 ; if arm-x is at limit, try adjusting arm-z
	 (if (eq (send self :move-safe arm toward :world t :bits bits) nil)
	     (dotimes (k stop)
	       (setq flag t)
	       (if (not (eq (send self :move-safe arm
				  (v+ toward (scale k adjust))
				  :world t :bits bits) nil))
		   (progn (setq flag nil)
			  (return))) ; if stretching succeeds, continue
	       )
	   )
	 ))
     )
  (:collision-check-all ()
     (let* ((upper-links (list (send self :larm :shoulder-y :child-link)
			       (send self :larm :elbow-p :child-link)
			       (send self :larm :wrist-y :child-link)
			       (send self :larm :wrist-r :child-link)
			       (send self :larm :t-1p :child-link)
			       (send self :larm :f-1p :child-link)
			       (send self :larm :f-2p :child-link)
			       (send self :larm :f-3p :child-link)
			       (send self :rarm :shoulder-y :child-link)
			       (send self :rarm :elbow-p :child-link)
			       (send self :rarm :wrist-y :child-link)
			       (send self :rarm :wrist-r :child-link)
			       (send self :rarm :t-1p :child-link)
			       (send self :rarm :f-1p :child-link)
			       (send self :rarm :f-2p :child-link)
			       (send self :rarm :f-3p :child-link)))
	    (lower-links  nil)
	    (check-upper nil)
	    (check-lower nil)
	    (result nil)
	    )
       (setq check-upper (send self :self-collision-check))
       (if (not (eq check-upper nil)) (print check-upper))
       (cond ((boundp '*aero-lower*)
	      (setq lower-links (list (elt (send *aero-lower* :torso :links) 0)
				      (send *aero-lower* :larm :shoulder-y :child-link)
				      (send *aero-lower* :larm :shoulder-p :child-link)
				      (send *aero-lower* :larm :elbow-p :child-link)
				      (send *aero-lower* :lleg :crotch-y :child-link)
				      (send *aero-lower* :lleg :crotch-p :child-link)
				      (send *aero-lower* :lleg :knee-p :child-link)
				      (send *aero-lower* :rarm :shoulder-y :child-link)
				      (send *aero-lower* :rarm :shoulder-p :child-link)
				      (send *aero-lower* :rarm :elbow-p :child-link)
				      (send *aero-lower* :rleg :crotch-y :child-link)
				      (send *aero-lower* :rleg :crotch-p :child-link)
				      (send *aero-lower* :rleg :knee-p :child-link)))
	      (dolist (x upper-links)
		(dolist (y lower-links)
		  (setq result (send self :self-collision-check :pairs (list (cons x y))))
		  (if (not (eq result nil))
		      (progn (print result)
			     (setq check-lower t)))
		  ))
	      ))
       (if (and (eq check-upper nil)
		(eq check-lower nil))
	   t
	 nil)
       )
     )
  )


(defclass aero-upper-interface
  :super rtm-ros-robot-interface
  :slots (actual-state
	  desired-state))
(defmethod aero-upper-interface
  (:init (&rest args)
    (send-super* :init :robot AeroUpperRobot-robot args)
    (mapcar #'(lambda (ctype)
                (send self :add-controller ctype))
            (send self :default-controller-list))
    (ros::roseus "aero_upper_interface")
    (ros::advertise "/aero_controller/command" trajectory_msgs::JointTrajectory)
    (ros::advertise "/aero_controller/wheel_command" trajectory_msgs::JointTrajectory)
    (ros::advertise "/aero_controller/wheel_servo" std_msgs::Bool)
    (ros::subscribe "/aero_controller/state" pr2_controllers_msgs::JointTrajectoryControllerState
		    #'send self :controller-callback 1)
    )
  (:default-controller-list ()
   (list :larm-controller
         :rarm-controller
         :torso-controller
         :fullbody-controller))
  (:default-controller ()
    (mapcar #'(lambda (ctype) (car (send self ctype))) (send self :default-controller-list)))
  (:fullbody-controller ()
   (send-message self robot-interface :default-controller))
  (:larm-controller ()
   (list
    (list
     (cons :controller-action "/larm_controller/follow_joint_trajectory_action")
     (cons :controller-state  "/larm_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "LARM_SHOULDER_P" "LARM_SHOULDER_R" "LARM_SHOULDER_Y" "LARM_ELBOW" "LARM_WRIST_Y" "LARM_WRIST_P")))))
  (:rarm-controller ()
   (list
    (list
     (cons :controller-action "/rarm_controller/follow_joint_trajectory_action")
     (cons :controller-state  "/rarm_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "RARM_SHOULDER_P" "RARM_SHOULDER_R" "RARM_SHOULDER_Y" "RARM_ELBOW" "RARM_WRIST_Y" "RARM_WRIST_P")))))
  (:torso-controller ()
   (list
    (list
     (cons :controller-action "/torso_controller/follow_joint_trajectory_action")
     (cons :controller-state  "/torso_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "WAIST_Y" "WAIST_P" "WAIST_R")))))
  (:angle-vector (av &optional (tm 5000))
     (let ((upper-joint-names (list "r_shoulder_pitch_joint"
  				    "r_shoulder_roll_joint"
  				    "r_elbow_yaw_joint"
  				    "r_elbow_pitch_joint"
  				    "r_wrist_roll_joint" ; yaw
  				    "r_wrist_top_joint"
  				    "r_wrist_bottom_joint"
  				    "r_hand_joint"
  				    "l_shoulder_pitch_joint"
  				    "l_shoulder_roll_joint"
  				    "l_elbow_yaw_joint"
  				    "l_elbow_pitch_joint"
  				    "l_wrist_roll_joint" ; yaw
  				    "l_wrist_top_joint"
  				    "l_wrist_bottom_joint"
  				    "l_hand_joint"
  				    "waist_pitch_joint" ; yaw
  				    "waist_right_joint"
  				    "waist_left_joint"
  				    "neck_yaw_joint"
  				    "neck_right_joint"
  				    "neck_left_joint"))
  	   (lower-joint-names (list "r_r_crotch_yaw_joint"
  				    "r_r_crotch_pitch_joint"
  				    "r_r_knee_pitch_joint"
  				    "f_r_crotch_yaw_joint"
  				    "f_r_crotch_pitch_joint"
  				    "f_r_knee_pitch_joint"
  				    "r_l_crotch_yaw_joint"
  				    "r_l_crotch_pitch_joint"
  				    "r_l_knee_pitch_joint"
  				    "f_l_crotch_yaw_joint"
  				    "f_l_crotch_pitch_joint"
  				    "f_l_knee_pitch_joint"))
  	   (type-check (length av)))
       (ros::publish "/aero_controller/command"
  		     (case type-check
  		       (28 (instance trajectory_msgs::JointTrajectory
  				     :init
  				     :joint_names upper-joint-names
  				     :points (list (instance trajectory_msgs::JointTrajectoryPoint
  							     :init
  							     :positions (concatenate float-vector
  										     (subseq av 0 8)
  										     (subseq av 11 19)
  										     (subseq av 22 25)
  										     (subseq av 25 28))
  							     :time_from_start (ros::time (/ tm 1000.0))))))
  		       (13 (instance trajectory_msgs::JointTrajectory
  				     :init
  				     :joint_names lower-joint-names
  				     :points (list (instance trajectory_msgs::JointTrajectoryPoint
  							     :init
  							     :positions (subseq av 0 12)
  							     :time_from_start (ros::time (/ tm 1000.0))))))
  		       (41 (instance trajectory_msgs::JointTrajectory
  				     :init
  				     :joint_names (append upper-joint-names lower-joint-names)
  				     :points (list (instance trajectory_msgs::JointTrajectoryPoint
  							     :init
  							     :positions (concatenate float-vector
  										     (subseq av 0 8)
  										     (subseq av 11 19)
  										     (subseq av 22 25)
  										     (subseq av 25 40))
  							     :time_from_start (ros::time (/ tm 1000.0))))))
  		       (t nil))))
     av)
  (:wheel-vector (av)
    (let ((wheel-joint-names (list "f_r_wheel_joint"
				   "r_r_wheel_joint"
				   "f_l_wheel_joint"
				   "r_l_wheel_joint")))
      (ros::publish "/aero_controller/wheel_command"
		    (instance trajectory_msgs::JointTrajectory
			      :init
			      :joint_names wheel-joint-names
			      :points (list (instance trajectory_msgs::JointTrajectoryPoint
						      :init
						      :positions av
						      :time_from_start (ros::time 0.0)))))
      av))
  (:wheel-on ()
    (ros::publish "/aero_controller/wheel_servo" (instance std_msgs::Bool :data t)))
  (:wheel-off ()
    (ros::publish "/aero_controller/wheel_servo" (instance std_msgs::Bool :data nil)))
  (:controller-callback (msg)
     (let ((i 0))
       (setq actual-state '())
       (setq desired-state '())
       (dolist (x (send msg :joint_names))
	 (setq actual-state (cons (cons x (elt (send msg :actual :positions) i)) actual-state))
	 (setq desired-state (cons (cons x (elt (send msg :desired :positions) i)) desired-state))
	 (setq i (+ i 1)))
       ))
  (:reference-vector (&optional (element nil))
     (ros::spin-once)
     (if (eq element nil)
	 (send self :stroke-to-angle desired-state)
       (elt (send self :stroke-to-angle desired-state) element)))
  (:actual-vector (&optional (element nil))
     (ros::spin-once)
     (if (eq element nil)
	 (send self :stroke-to-angle actual-state)
       (elt (send self :stroke-to-angle actual-state) element)))
  )


;; (defun aero-upper-init (&rest args)
;;   (if (not (boundp '*ri-upper*))
;;       (setq *ri-upper* (instance* aero-upper-interface :init args)))
;;   (if (not (boundp '*aero-upper*))
;;       (setq *aero-upper* (instance AeroUpperRobot-robot :init)))
;;   )

(defvar *base-time* 1000)
(defvar *real* nil)
(defun send-angle-vector
  (&key (angle-vector (send *aero* :stroke-vector)) (time (* 5 *base-time*)))
  (unless (boundp '*aero*)
    (warn ";; create *aero* first~S") (return))
  ;; (unless (boundp '*ri*) (unix:usleep time))
  (when *real*
    (unless (boundp '*ri*)
      (warn ";; create *ri* first~S") (return))
    (send *ri* :angle-vector angle-vector time))
  )

(defun move-hand-vi
  (&key (robot *aero-upper*) (real *real*) (arm :rarm)
	(move-time (* 0.5 *base-time*)) (move-dist 20)
	(move-headp t) (draw-objects t))
  (let (key-input
	velocity (short-dist (/ move-dist 3.0))
	(forward-dir (float-vector 1 0 0))
	(left-dir (float-vector 0 1 0))
	(up-dir (float-vector 0 0 1)))
    (when move-headp
      (send robot :head :look-at (send robot arm :end-coords :worldpos)))
    (when real
      (send-angle-vector :time (* 3 *base-time*)))
    (while t
      (setf key-input (read-line))
      (cond
       ((equal key-input "j")
	(setf velocity (scale (- move-dist) up-dir))) ;; "j"=down
       ((equal key-input "k")
	(setf velocity (scale move-dist up-dir))) ;; "k"=up
       ((equal key-input "h")
	(setf velocity (scale move-dist left-dir))) ;; "h"=left
       ((equal key-input "l")
	(setf velocity (scale (- move-dist) left-dir))) ;; "l"=right
       ((equal key-input "f")
	(setf velocity (scale move-dist forward-dir))) ;; "f"=forward
       ((equal key-input "b")
	(setf velocity (scale (- move-dist) forward-dir))) ;; "b"=backward

       ((equal key-input "jj")
	(setf velocity (scale (- short-dist) up-dir))) ;; "j"=down
       ((equal key-input "kk")
	(setf velocity (scale short-dist up-dir))) ;; "k"=up
       ((equal key-input "hh")
	(setf velocity (scale short-dist left-dir))) ;; "h"=left
       ((equal key-input "ll")
	(setf velocity (scale (- short-dist) left-dir))) ;; "h"=left
       ((equal key-input "ff")
	(setf velocity (scale short-dist forward-dir))) ;; "f"=forward
       ((equal key-input "bb")
	(setf velocity (scale (- short-dist) forward-dir))) ;; "b"=backward

       ((equal key-input "q") (return-from move-hand-vi)) ;; "q"=quit
       )
      (when (send robot arm :move-end-pos velocity :world :rotation-axis :z)
	(when move-headp
	  (send robot :head :look-at (send robot arm :end-coords :worldpos)))
	(if draw-objects (send *irtviewer* :draw-objects))
	(when real
	  (send-angle-vector :time move-time)))
      )
    ))

(defun rot-hand-vi
  (&key (robot *aero-upper*) (real *real*) (arm :rarm)
	(move-time (* 0.5 *base-time*)) (move-ang 5)
	(move-headp t) (draw-objects t))
  (let (key-input rot-axis)
    (when move-headp
      (send robot :head :look-at (send robot arm :end-coords :worldpos)))
    (when real
      (send-angle-vector :time (* 3 *base-time*)))
    (while t
      (setf key-input (read-line))
      (cond
       ((equal key-input "j") (setf rot-axis :z)) ;; z
       ((equal key-input "k") (setf rot-axis :-z)) ;; -z
       ((equal key-input "h") (setf rot-axis :-x)) ;; x
       ((equal key-input "l") (setf rot-axis :x)) ;; -x
       ((equal key-input "f") (setf rot-axis :y)) ;; y
       ((equal key-input "b") (setf rot-axis :-y)) ;; -y
       ((equal key-input "reset")
	(send robot arm :inverse-kinematics
	      (make-coords :pos (send robot arm :end-coords :worldpos))
	      :rotation-axis :z)
	(if draw-objects (send *irtviewer* :draw-objects))
	(when real
	  (send-angle-vector :time (* 3 *base-time*)))
	(return-from rot-hand-vi))
       ((equal key-input "q") (return-from rot-hand-vi)) ;; "q"=quit
       )
      (when (send robot arm :move-end-rot move-ang rot-axis :local)
	(when move-headp
	  (send robot :head :look-at (send robot arm :end-coords :worldpos)))
	(if draw-objects (send *irtviewer* :draw-objects))
	(when real
	  (send-angle-vector :time move-time)))
      )
    ))
