(ros::roseus-add-msgs "sensor_msgs")

(unless (boundp '*aero*)
  (load "package://aero_ros_bridge/euslisp/aero-interface.l")
  (aero-init)
  (send *aero* :reset-manip-pose)
  (send *aero* :draw-on))

(setq *points* nil)
(setq *head-co* (send (car (last (send *aero* :upper :head))) :worldcoords))
(setq *camera-co*
      (make-cascoords :pos (v+ (send *head-co* :worldpos) #f(60 0 80))
		      :rpy (list -pi/2 0 -pi/2)))
(send (car (last (send *aero* :upper :head))) :assoc *camera-co*)

(defun callback (msg)
  ;; ROSのメッセージをeuslispのpointcloudに変換
  (setq *points*
        (make-eus-pointcloud-from-ros-msg
         msg :remove-nan t)) ;; for replace -> :remove-nan :replace
  (if (and *points* (> (send *points* :size) 0))
      (setf (get *points* :header) (send msg :header))
    (setq *points* nil))
  *points*)

(ros::subscribe "/stereo/points2"
                sensor_msgs::PointCloud2
                #'callback)

(defun update-pointcloud ()
  (setq *points* nil)
  (ros::spin-once)
  (when *points*
    (send *points* :newcoords (send *camera-co* :copy-worldcoords))
    (objects *points*)
    )
  )
