(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :aero "package://aero_ros_bridge/models/aero.l")

(defclass aero-interface
  :super rtm-ros-robot-interface
  :slots ())
(defmethod aero-interface
  (:init (&rest args)
    (send-super* :init :robot AeroRobot-robot args)
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

(defun aero-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* aero-interface :init args)))
  (if (not (boundp '*sr*))
      (setq *aero* (instance AeroRobot-robot :init)))
  )
