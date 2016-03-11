;; @file recog.l
;; @brief recognition functions for IREX demo

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; @brief call /color_memorizer/find_match
(defun set-object (&key (timeout 60))
  (let (req res
	(h_max_v nil) (h_min_v nil)
	(s_max_v nil) (s_min_v nil)
	(i_max_v nil) (i_min_v nil)
	)
    (setq req (instance aero_startup::PointHSIArrayRequest :init))
    (send req :size (length *samples*))
    (dolist (x *samples*)
      (push (car (elt x 1)) h_min_v) (push (cadr (elt x 1)) h_max_v)
      (push (car (elt x 2)) s_min_v) (push (cadr (elt x 2)) s_max_v)
      (push (car (elt x 3)) i_min_v) (push (cadr (elt x 3)) i_max_v)
      )
    (nreverse h_min_v) (nreverse h_max_v)
    (nreverse s_min_v) (nreverse s_max_v)
    (nreverse i_min_v) (nreverse i_max_v)
    (send req :h_cap_v h_max_v) (send req :h_v h_min_v)
    (send req :s_cap_v (coerce s_max_v string))
    (send req :s_v (coerce s_min_v string))
    (send req :i_cap_v (coerce i_max_v string))
    (send req :i_v (coerce i_min_v string))
    (send req :time timeout)
    (setq res (ros::service-call "/color_memorizer/find_match" req))
    (send res :status) ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; @brief call /point_cloud/perception_area
;; @param hsi ((h s i) (h_cap s_cap i_cap))
;; @param xyz ((x y z) (x_cap y_cap z_cap))
(defun find-object (hsi xyz &optional (precise t))
  (let (req res
	)
    (setq req (instance aero_startup::PointXYZHSIRequest :init))
    (send req :x (car (elt xyz 0))) (send req :x_cap (cadr (elt xyz 0)))
    (send req :y (car (elt xyz 1))) (send req :y_cap (cadr (elt xyz 1)))
    (send req :z (car (elt xyz 2))) (send req :z_cap (cadr (elt xyz 2)))
    (send req :h (car (elt hsi 1))) (send req :h_cap (cadr (elt hsi 1)))
    (send req :s (car (elt hsi 2))) (send req :s_cap (cadr (elt hsi 2)))
    (send req :i (car (elt hsi 3))) (send req :i_cap (cadr (elt hsi 3)))
    (send req :precise precise)
    ;; (setq res (ros::service-call "/extract_object_features/perception_area" req))
    (setq res (ros::service-call "/point_cloud/perception_area" req))
    (send res :status) ))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; @brief call "str" service
;; @param xyz ((x y z) (x_cap y_cap z_cap))
;; @param str service name
(defun reconfigure (xyz str)
  (let (req res
        )
    (setq req (instance aero_startup::PointXYZRequest :init))
    (send req :x (car (elt xyz 0))) (send req :x_cap (cadr (elt xyz 0)))
    (send req :y (car (elt xyz 1))) (send req :y_cap (cadr (elt xyz 1)))
    (send req :z (car (elt xyz 2))) (send req :z_cap (cadr (elt xyz 2)))
    (setq res (ros::service-call str req))
    (send res :status) ))
