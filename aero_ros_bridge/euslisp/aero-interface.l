(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :aero-upper "package://aero_ros_bridge/euslisp/aero-upper-interface.l")
(require :aero-lower "package://aero_ros_bridge/euslisp/aero-lower-interface.l")
(load "package://aero_ros_bridge/euslisp/aero-upper-strokes.l")
(load "package://aero_ros_bridge/euslisp/aero-upper-angles.l")



(defun aero-init (&rest args)
  (aero-upper-init) ;; this inits the model (*aero-upper*) and simulator (*ri*) for the upper body
  (if (not (boundp '*aero-lower*)) ;; this inits the model (*aero-lower*) only for the lower body
      (setq *aero-lower* (instance AeroLowerRobot-robot :init)))
  ;; move aero-upper to top location of aero-lower
  (if (boundp '*aero-lower*)
      (send *aero-upper* :newcoords
            (send (send *aero-lower* :torso :end-coords)
                  :worldcoords))
    )
  ;; (send *aero-upper* :translate (v- (send *aero-upper* :worldpos)))
  ;; (send *aero-upper* :translate (send *aero-lower* :links :WAIST_LINK0 :worldpos)
  (send (send *aero-lower* :links :WAIST_LINK0) :assoc (send *aero-upper* :links :WAIST_LINK0))
  )

;; calculate center of mass of the whole robot using center of mass of aero upper and aero lower
(defun get-com ()
  (scale (/ 1 (+ (send *aero-upper* :weight) (send *aero-lower* :weight)))
                                (v+ (scale (send *aero-upper* :weight) (send *aero-upper* :centroid))
                                    (scale (send *aero-lower* :weight) (send *aero-lower* :centroid))))
  )

;; helper function for proejecting a vector onto ground plane
(defun project-on-ground (in_coords)
    (float-vector (aref in_coords 0) (aref in_coords 1) 0)
  )

;; draw center of mass projected onto ground plane
(defun draw-centroid-on-ground ()
  (let ((com-on-ground (make-coords :pos (project-on-ground (get-com))))
        )
    (send com-on-ground :draw-on :flush t)
    com-on-ground)  
  )

(defun draw-upper-centroid-on-ground ()
  (let ((com-on-ground (make-coords :pos (project-on-ground (send *aero-upper* :centroid))))
        )
    (send com-on-ground :draw-on :flush t)
    com-on-ground)  
  )

(defun draw-lower-centroid-on-ground ()
  (let ((com-on-ground (make-coords :pos (project-on-ground (send *aero-lower* :centroid))))
        )
    (send com-on-ground :draw-on :flush t)
    com-on-ground)  
  )

