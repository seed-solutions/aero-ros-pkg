#!/usr/bin/env roseus

(ros::load-ros-manifest "aero_startup")

(ros::roseus "hand_interface")

(ros::wait-for-service "/aero_hand_controller")

(defmethod aero-upper-interface
  (:hand-angle (arm robot)
    (let (av)
      (send self :state)
      (setq av (send self :actual-vector))
      (cond ((eq arm :rarm)
	     (progn (send robot :rarm :t-1p (elt av 7))
		    (send robot :rarm :f-1p (- (els av 7)))))
	    ((eq arm :larm)
	     (progn (send robot :larm :t-1p (elt av 18))
		    (send robot :larm :f-1p (- (els av 18)))))
	    ((eq arm :arms)
	     (progn (send robot :rarm :t-1p (elt av 7))
		    (send robot :rarm :f-1p (- (els av 7)))
		    (send robot :larm :t-1p (elt av 18))
		    (send robot :larm :f-1p (- (els av 18)))))
	    )
      ))
  (:grasp (arm &optional robot &key (tfail 0.8) (twarn -0.9))
    (let (req
	  res
	  )
      (setq req (instance
		 aero_startup::AeroHandControllerRequest
		 :init))
      (cond ((eq arm :larm) (send req :hand "left"))
	    ((eq arm :rarm) (send req :hand "right"))
	    ((eq arm :arms) (send req :hand "both"))
	    )
      (send req :command "grasp")
      (send req :thre_fail tfail)
      (send req :thre_warn twarn)
      (setq res (ros::service-call "/aero_hand_controller" req))
      (when robot (send self :hand-angle arm robot))
      (print (send res :status))
      ))
  (:ungrasp (arm &optional robot)
    (let (req
	  res
	  )
      (setq req (instance
		 aero_startup::AeroHandControllerRequest
		 :init))
      (cond ((eq arm :larm) (send req :hand "left"))
	    ((eq arm :rarm) (send req :hand "right"))
	    ((eq arm :arms) (send req :hand "both"))
	    )
      (send req :command "ungrasp")
      (setq res (ros::service-call "/aero_hand_controller" req))
      (when robot (send self :hand-angle arm robot))
      (print (send res :status))
      ))
  (:grasp-angle (arm angle &optional robot)
;; rarm :  open-limit 50 close-limit -50
;; larm :  open-limit -50  close-limit 50

    (let (req
	  res
	  )
      (setq req (instance
		 aero_startup::AeroHandControllerRequest
		 :init))
      (cond ((eq arm :larm) (progn (send req :hand "left") (send req :larm_angle angle)))
	    ((eq arm :rarm) (progn (send req :hand "right") (send req :rarm_angle angle)))
	    )
      (send req :command "grasp-angle")
      (setq res (ros::service-call "/aero_hand_controller" req))
      (when robot (send self :hand-angle arm robot))
      (print (send res :status))
      ))
  );; end of defmethod aero-upper-interface
