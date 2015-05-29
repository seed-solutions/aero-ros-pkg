(ros::advertise "/head_pos" geometry_msgs::PoseStamped 1)
(defun broadcast-aero-head-tf ()
  (let (world-head-coords world-waist-coords waist-head-coords pose)
    (setq world-head-coords
	  (send (car (last (send *aero* :upper :head))) :copy-worldcoords))
    (setq world-waist-coords
	  (send (car (send *aero* :upper :links)) :copy-worldcoords))
    (setq waist-head-coords
	  (send (send world-waist-coords :inverse-transformation)
		:transform world-head-coords))
    (setq pose
	  (ros::coords->tf-pose-stamped
	   waist-head-coords
	   "base_link"))
    (ros::publish "/head_pos" pose)))





