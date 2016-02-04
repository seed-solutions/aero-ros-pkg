#!/usr/bin/env roseus

(ros::load-ros-manifest "aero_startup")
(load "package://aeroeus/aero-interface.l")
(load "kiva-pod-model.l")
(load "tote-model.l")

(setq *real* nil)

(unless (boundp '*kiva-pod*)
  (setq *kiva-pod* (make-kiva-pod))
  (send *kiva-pod* :translate (float-vector 1000 0 0))
  (send *kiva-pod* :rotate pi :z))

(unless (boundp '*tote*)
  (setq *tote* (make-tote))
  (send *tote* :translate (float-vector 0 400 700)))

(unless (boundp '*aero*)
  (if *real*
      (progn
	(aero-init)
	(load-controllers))
    (progn
      (setq *aero* (instance AeroUpperRobot-robot :init))))
  (send *aero* :translate (float-vector 0 0 850)))

(objects (flatten (list *aero* *kiva-pod* *tote*)))


(defun ik-with-torso (coords)
  (when
      (send *aero* :rarm :inverse-kinematics coords
	    :rotation-axis :z :torso t
	    :link-list
	    (send *aero* :link-list
		  (send *aero* :rarm :end-coords :parent)
		  (car (send *aero* :torso :links))))
    (send *aero* :head :look-at (send *aero* :rarm :end-coords :worldpos))
    ))

;; example
;; (ik-with-torso (send (send *kiva-pod* :spot-h :copy-worldcoords) :translate (float-vector 0 0 100)))
