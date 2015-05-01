(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :aero "package://aero_ros_bridge/models/aero_upper.l")

(defun hand-poses (hand args)
  (let ((pose (car args))
	(angle 0)
	(hand-vector (float-vector
		      (send *robot* hand :t-1p :joint-angle)
		      (send *robot* hand :f-1p :joint-angle)
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
    (send *robot* hand :t-1p :joint-angle (elt hand-vector 0))
    (send *robot* hand :f-1p :joint-angle (elt hand-vector 1))
    (send *robot* hand :f-2p :joint-angle (elt hand-vector 2))
    (send *robot* hand :f-3p :joint-angle (elt hand-vector 3))
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
  )

(defclass aero-upper-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod aero-upper-interface
  (:init (&rest args)
    (send-super* :init :robot AeroUpperRobot-robot args)
    (mapcar #'(lambda (ctype)
                (send self :add-controller ctype))
            (send self :default-controller-list))
    )
  (:default-controller-list ()
   (list :larm-controller
         :rarm-controller
         :torso-controller
         :lhand-controller
         :rhand-controller
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
     (cons :joint-names (list "WAIST_P" "WAIST_R" "CHEST")))))
  (:lhand-controller ()
   (list
    (list
     (cons :controller-action "/lhand_controller/follow_joint_trajectory_action")
     (cons :controller-state  "/lhand_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "LARM_WRIST_R")))))
  (:rhand-controller ()
   (list
    (list
     (cons :controller-action "/rhand_controller/follow_joint_trajectory_action")
     (cons :controller-state  "/rhand_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "RARM_WRIST_R")))))
  )

(defun aero-upper-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* aero-upper-interface :init args)))
  (if (not (boundp '*aero-upper*))
      (setq *aero-upper* (instance AeroUpperRobot-robot :init)))
  (setq *robot* *aero-upper*)
  )
