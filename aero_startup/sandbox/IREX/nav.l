;; @file nav.l
;; @brief base navigation functions for IREX demo

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun go-for-target (xy ~xy)
  (let (req res
	)
    (setq req (instance aero_startup::ObjectGoXYZHSIRequest :init))
    (send req :object "object_l")
    (send req :go_x (elt xy 0)) ;; any large number if x movement
    (send req :go_y (elt xy 1)) ;; any large number if y movement
    (send req :until_x (elt ~xy 0)) ;; stop distance, large number for no limits
    (send req :until_y (elt ~xy 1)) ;; stop distance, large number for no limits
    (setq res (ros::service-call "/go_run/target_object" req))
    (list (send res :status) (send res :time)) ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun go-for-time (x y theta time)
  (let (req res
	)
    (setq req (instance aero_startup::GoTimeRequest :init))
    (send req :go_x x)
    (send req :go_y y)
    (send req :go_theta theta)
    (send req :time time)
    (setq res (ros::service-call "/go_run/target_time" req))
    (send res :status) ))
