#!/usr/bin/env roseus

;; @in "/look_at/target" geometry_msgs::Point
;; @out "/aero_controller/command"
;; when Point has position, keep Aero looking at there
;; when Point is (0,0,0), reset head pose

(ros::load-ros-manifest "roseus")
(ros::load-ros-package "geometry_msgs")
(ros::load-ros-package "trajectory_msgs")

(ros::roseus "look_at_node")

(load "package://aeroeus/aero-interface.l")


(setq *aero* (instance AeroUpperRobot-robot :init))
(send *aero* :reset-manip-pose)
(setq ((send *aero* :head :neck-y) . min-angle) -40)
(setq ((send *aero* :head :neck-y) . max-angle) 40)
(setq ((send *aero* :head :neck-r) . max-angle) 0)
(setq ((send *aero* :head :neck-r) . min-angle) 0)

(setf x::*display* 0)
(setq *ri* (instance aero-upper-interface :init))


(setq *reset-head-once-flag* t)
(setq *target* (instance geometry_msgs::Point :init))
(setq *lifter* (instance geometry_msgs::Point :init))


(defun look-at-node (msg)
  (setq *target* msg)
  (print "look-at")
)

(defun lifter-callback (msg)
  (setq *lifter* msg)
)


(ros::subscribe "/look_at/target"
                        geometry_msgs::Point #'look-at-node)
(ros::subscribe "/lifter_position"
                        geometry_msgs::Point #'lifter-callback)
(ros::advertise "/aero_controller/command" trajectory_msgs::JointTrajectory 1)
(unix::sleep 1);; sleep is needed when the time is short between creating publisher and publishing



(do-until-key
 (ros::spin-once)
 (setq *head_msg* (instance trajectory_msgs::JointTrajectory :init))
 (send *head_msg* :joint_names (list "neck_y_joint" "neck_p_joint" "neck_r_joint"))

 (if (and (= (send *target* :x) 0.0) (= (send *target* :y) 0.0) (= (send *target* :z) 0.0)) 
     (when *reset-head-once-flag*
       (print "reset head")
       (send *head_msg* :points 
             (list (instance trajectory_msgs::JointTrajectoryPoint :init 
                             :positions (list 0 0 0) 
                             :time_from_start (ros::time 1))))
       (setq *reset-head-once-flag* nil)
       (ros::publish "/aero_controller/command" *head_msg*))
   (progn
     (send *aero* :angle-vector (send *ri* :actual-vector))
     (send *aero* :head :look-at 
           (float-vector 
            (* 1000 (- (send *target* :x) (send *lifter* :x))) 
            (* 1000 (send *target* :y)) 
            (* 1000 (- (send *target* :z) (+ (send *lifter* :z) 0.58))) )
           )
     (send *head_msg* 
           :points 
           (list 
            (instance trajectory_msgs::JointTrajectoryPoint :init 
                      :positions (list 
                                  (max (min (deg2rad (send *aero* :head :neck-y :joint-angle)) 0.8) -0.8)
                                  (max (min (deg2rad (send *aero* :head :neck-p :joint-angle)) 0.8) -0.8) 
                                  (max (min (deg2rad (send *aero* :head :neck-r :joint-angle)) 0.8) -0.8)) 
                      :time_from_start (ros::time 0.5)))
           )
     (ros::publish "/aero_controller/command" *head_msg*)
     (setq *reset-head-once-flag* t))
   )
 (unix::usleep (* 1000 500))
)
