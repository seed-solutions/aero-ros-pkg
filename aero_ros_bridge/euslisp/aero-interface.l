(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require :aero "package://aero_ros_bridge/euslisp/aero.l")

(defclass aero-interface
  :super object
  :slots ())
(defmethod aero-interface
  (:init (&key args)
    )
  )
  

(defun aero-init (&rest args)
  (when (not (boundp '*ri*))
    (let ((disp (copy-object x::*display*)))
      (setf x::*display* 0)
      (setq *ri* (instance* aero-upper-interface :init args))
      (setf x::*display* disp)
      )) ;; currently communication with whole robot is implemented in aero-upper-interface
  (if (not (boundp '*aero*))
      (setq *aero* (instance AeroRobot-robot :init)))
  )


