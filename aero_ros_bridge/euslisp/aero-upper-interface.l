#!/usr/bin/env roseus
(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :aero-upper "package://aero_ros_bridge/models/aero_upper.l")
(load "package://aero_ros_bridge/euslisp/aero-upper-strokes.l")

(ros::load-ros-manifest "trajectory_msgs")


(defun hand-poses (hand args)
  (let ((pose (car args))
	(angle 0)
	(hand-vector (float-vector
		      (send *aero-upper* hand :t-1p :joint-angle)
		      (send *aero-upper* hand :f-1p :joint-angle)
		      0 0))
	)
    (case pose
      (:reset-pose (setq hand-vector #f(0 0 0 0)))
      (:open-pose (if (eq hand :larm) (setq hand-vector #f(-50 50 0 0))
		    (setq hand-vector #f(50 -50 0 0))))
      (:close-pose (if (eq hand :larm) (setq hand-vector #f(50 -50 0 0))
		     (setq hand-vector #f(-50 50 0 0))))
      (:joint-angle (progn
		      (setq angle (elt (cdr args) 0))
		      (if (eq hand :larm)
			  (setq hand-vector
				(float-vector angle (- angle) 0 0))
			(setq hand-vector
			      (float-vector (- angle) angle 0 0))
			)
		      ))
      (t (return-from :hand-poses nil))
      )
    (send *aero-upper* hand :t-1p :joint-angle (elt hand-vector 0))
    (send *aero-upper* hand :f-1p :joint-angle (elt hand-vector 1))
    (send *aero-upper* hand :f-2p :joint-angle (elt hand-vector 2))
    (send *aero-upper* hand :f-3p :joint-angle (elt hand-vector 3))
    )
  )

(defmethod AeroUpperRobot-robot
  (:hand (&rest args)
    (if (null args) (return-from :hand nil))
     (let ((key (car args))
           (nargs (cdr args)))
       (unless (keywordp key)
         (return-from :hand nil))
       (case key
	 (:arms (progn (hand-poses :larm nargs) (hand-poses :rarm nargs)))
	 (:larm (hand-poses :larm nargs))
	 (:rarm (hand-poses :rarm nargs))
	 (t nil)
	 )
       )
     )
  (:fix-upper-to-coords (&rest coords)
     (if (null coords)
	 (if (boundp '*aero-lower*)
	     (send self :newcoords
		   (send (send *aero-lower* :torso :end-coords)
			 :worldcoords))
	   )
       (send self :newcoords (car coords))
       )
     )
  (:move-safe (arm to &key (bits 10)
			   (world nil))
     (let* ((delta-to 0)
	    (coords :local)
	    )
       (if (eq world t) (setq coords :world))
       (setq delta-to (scale (/ 1.0 bits) to))
       (dotimes (i bits)
	 (send self arm :move-end-pos delta-to coords) ; move a step
	 (cond ((not (send self :collision-check-all)) ; if collides
		(send self arm :move-end-pos (scale (- 1) delta-to) coords) ; go back a step 
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

;; (defclass AeroUpperRobot-robot-collision
;;   :super AeroUpperRobot-robot
;;   :slots ())
;; (defmethod AeroUpperRobot-robot-collision
;;   (:init (&rest args)
;; 	 (send-super* :init args)
;; 	 (setq collision-avoidance-links (list (send self :torso :waist-p :parent-link)
;; 					       (send self :torso :waist-r :child-link)
;; 					       (send self :head :neck-r :child-link)
;; 					       (send self :rarm :shoulder-y :child-link)
;; 					       (send self :rarm :elbow-p :child-link)
;; 					       (send self :rarm :wrist-y :child-link)
;; 					       (send self :rarm :wrist-r :child-link)
;; 					       (send self :larm :shoulder-y :child-link)
;; 					       (send self :larm :elbow-p :child-link)
;; 					       (send self :larm :wrist-y :child-link)
;; 					       (send self :larm :wrist-r :child-link)
;; 					       ))
;; 	 (setq larm-collision-avoidance-links collision-avoidance-links)
;; 	 (setq rarm-collision-avoidance-links collision-avoidance-links)
;; 	 )
;;   )

(defclass aero-upper-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod aero-upper-interface
  (:init (&rest args)
    (send-super* :init :robot AeroUpperRobot-robot args)
    (mapcar #'(lambda (ctype)
                (send self :add-controller ctype))
            (send self :default-controller-list))
    (ros::roseus "aero_upper_interface")
    (ros::advertise "command" trajectory_msgs::JointTrajectory)
    )
  (:default-controller-list ()
   (list :larm-controller
         :rarm-controller
         :torso-controller
         ;; :lhand-controller
         ;; :rhand-controller
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
     (ros::publish "command"
		   (instance trajectory_msgs::JointTrajectory
			     :init
			     :joint_names (list "r_shoulder_pitch_joint"
						"r_shoulder_roll_joint"
						"r_elbow_yaw_joint"
						"r_elbow_pitch_joint"
						"r_wrist_yaw_joint" ; missing
						;"r_wrist_roll_joint"
						"r_wrist_top_joint"
						"r_wrist_bottom_joint"
						"r_hand_joint"
						"l_shoulder_pitch_joint"
						"l_shoulder_roll_joint"
						"l_elbow_yaw_joint"
						"l_elbow_pitch_joint"
						"l_wrist_yaw_joint" ; missing
						;"l_wrist_roll_joint"
						"l_wrist_top_joint"
						"l_wrist_bottom_joint"
						"l_hand_joint"
						"waist_yaw_joint" ; missing
						;"waist_pitch_joint"
						"waist_right_joint"
						"waist_left_joint"
						"neck_yaw_joint"
						"neck_right_joint"
						"neck_left_joint")
			     :points (list (instance trajectory_msgs::JointTrajectoryPoint
						     :init
						     :positions (concatenate float-vector
									     (subseq av 0 8)
									     (subseq av 11 19)
									     (subseq av 22 25)
									     (subseq av 25 28))
						     :time_from_start (ros::time 0)))))
     av)     
     
  ;; (:lhand-controller ()
  ;;  (list
  ;;   (list
  ;;    (cons :controller-action "/lhand_controller/follow_joint_trajectory_action")
  ;;    (cons :controller-state  "/lhand_controller/state")
  ;;    (cons :action-type control_msgs::FollowJointTrajectoryAction)
  ;;    (cons :joint-names (list "LARM_WRIST_R")))))
  ;; (:rhand-controller ()
  ;;  (list
  ;;   (list
  ;;    (cons :controller-action "/rhand_controller/follow_joint_trajectory_action")
  ;;    (cons :controller-state  "/rhand_controller/state")
  ;;    (cons :action-type control_msgs::FollowJointTrajectoryAction)
  ;;    (cons :joint-names (list "RARM_WRIST_R")))))
  )

(defun aero-upper-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* aero-upper-interface :init args)))
  (if (not (boundp '*aero-upper*))
      (setq *aero-upper* (instance AeroUpperRobot-robot :init)))
  )
