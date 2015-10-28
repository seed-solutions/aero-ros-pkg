#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(defvar *aero-ik-target* nil)

(defun callback (msg)
  (setq *aero-ik-target* (send msg :data)))

(ros::advertise "/aero_ik" std_msgs::Float32MultiArray 1)
(ros::subscribe "/aero_ik_res" std_msgs::Float32MultiArray #'callback)


(defun aero-ik (arm coords)
  (let (msg
	get-coords
	)
    (setq msg (instance std_msgs::Float32MultiArray :init))
    (send msg :data (list (elt (send coords :pos) 0)
			  (elt (send coords :pos) 1)
			  (elt (send coords :pos) 2)
			  (send *aero-upper* :head :neck-p :joint-angle)
			  (send *aero-upper* :head :neck-y :joint-angle)))
    (ros::publish "/aero_ik" msg)
    (unix::usleep (* 1000 1000))
    (ros::spin-once)
    (setq get-coords
	  (make-coords :pos (v+ (send coords :pos)
				(v- (send coords :pos) *aero-ik-target*))
		       :rot (send coords :rot)))
    (if (eq (send *aero-upper* arm :inverse-kinematics get-coords) nil)
	(send *aero-upper* arm :inverse-kinematics get-coords
	      :revert-if-fail nil)
      )
    get-coords))

(load "package://aero_ros_bridge/euslisp/tf.l")

(defun go-for-object (arm)
  (let (mt
	)
    (send *aero* :ungrasp arm)
    (aero-ik arm (make-coords :pos (send (~tf arm) :pos)))
    (send *irtviewer* :draw-objects)
    (setq mt (make-coords :pos (send (send *aero-upper* :larm :end-coords
					   :copy-worldcoords) :pos)
			  :rot (send (~tf arm) :rot)))
    (if (eq (send *aero-upper* arm :inverse-kinematics mt) nil)
	(send *aero-upper* arm :inverse-kinematics mt
	      :revert-if-fail nil)
      )
    mt))
