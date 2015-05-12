(defun testsr ()
  (load "package://aero_ros_bridge/euslisp/samplerobot-interface.l")
  (samplerobot-init)
  (pprint (send *sr* :slots))
  (print "")
  (print "")
  (print "")
  (send *sr* :torso)
)

(defun testaero ()
  (load "/home/wesleyc/docs/catkin_ws/src/aero_ros_bridge/euslisp/aero-interface.l")
  ;(load "package://robot_model_compiler/euslisp/aero-stick-figure-interface.l")
  (aero-init)
  (objects *aero*)
;  (pprint (send *aero* :slots))
;  (print "")
;  (print "")
;  (print "")
;  (send *aero* :torso)
)

(defun testaerolower ()
  (load "package://aero_ros_bridge/euslisp/aero-lower-interface.l")
  ;(load "package://robot_model_compiler/euslisp/aero-stick-figure-interface.l")
  (aero-lower-init)
)

(defun moveRobot (&optional (time 3000))
      (send *ri-lower* :angle-vector (send *aero-lower* :angle-vector) time))

(defun test-aero-motion ()

  (send *aero-lower* :rarm :shoulder-y :joint-angle 30)
  (send *aero-lower* :larm :shoulder-y :joint-angle -30)
  (send *aero-lower* :rleg :crotch-y :joint-angle -30)
  (send *aero-lower* :lleg :crotch-y :joint-angle 30)
  
  (send *irtviewer* :draw-objects)
  (moveRobot)
  (send *ri-lower* :wait-interpolation "or")
              

  
  (send *aero-lower* :rarm :shoulder-p :joint-angle -45)
  (send *aero-lower* :larm :shoulder-p :joint-angle -45)
  (send *aero-lower* :rleg :crotch-p :joint-angle -45)
  (send *aero-lower* :lleg :crotch-p :joint-angle -45)
  
  (send *irtviewer* :draw-objects)
  (moveRobot)
  (send *ri-lower* :wait-interpolation "or")


  (send *aero-lower* :rarm :elbow-p :joint-angle 45)
  (send *aero-lower* :larm :elbow-p :joint-angle 45)
  (send *aero-lower* :rleg :knee-p :joint-angle 45)
  (send *aero-lower* :lleg :knee-p :joint-angle 45)

  (send *irtviewer* :draw-objects)
  (moveRobot)
  (send *ri-lower* :wait-interpolation "or")    
  
  (send *aero-lower* :reset-pose)
  (send *irtviewer* :draw-objects)
  (moveRobot)

)


(testaerolower)
(objects *aero-lower*)
;; (pprint (send *aero-lower* :slots))
(test-aero-motion)

