(load "package://aero_ros_bridge/euslisp/aero-interface.l")

(defun relax-arms (&key (part :all))
  (let ((iterations 5)
        (waitTime 1))
    (setq larmJointIds `(0 1 2 3 4 5 6))
    (setq rarmJointIds `(11 12 13 14 15 16 17))
    (case part
          (:rarm 
           (setq jointIds larmJointIds))
          (:larm
           (setq jointIds rarmJointIds))
          (otherwise
           (setq jointIds (append rarmJointIds larmJointIds))
           )
          )
    (dotimes (i 5)
      (setq acutal-vector (send *ri* :actual-vector))
      (setq model-vector (send *aero* :angle-vector))
      
      (dolist (x jointIds)
        (setf (aref model-vector x) (aref actual-vector x))
        )
      
               
      (send *aero* :angle-vector model-vector)
      (send-angle-vector 100)
      (format t "relax-arms -- iteration ~d out of ~d~%" i iteration)
      (format t "sleep ~d~%")
      (sleep sleepTime)
      )
    )
  )

(defun isArmInContact (arm)
  (setq armContactErrorThre 10)
  (format t "isArmInContact -- error sum ~a threshold ~a~%" (send *aero* :upper-error-sum arm) armContactErrorThre)
  (if (> (send *aero* :upper-error-sum arm) armContactErrorThre)
      t
    nil)
  )

(defun reach-until-contact (arm goal)
  (let* ((goal-direction (normalize-vector goal))
        (goal-distance (norm goal))
        (step-size 5)
        (remainingDist goal-distance))
    (while (and (not (isArmInContact arm))
                (> remainingDist 0))
      (send *aero* :upper arm :move-end-pos (scale step-size goal-direction) :local :rotation-direction nil) 
      (send-angle-vector :time 100)
      (format t ":reach-until-contact loop remaining distance ~f~%" remainingDist)
      (unix:sleep 1)
      (setq remainingDist (- remainingDist step-size))
      )    
    )
  )

(defun reach-until-grasped (arm goal (&key hand-grasp-range #f(-40 0)))
  (let* ((goal-direction (normalize-vector goal))
        (goal-distance (norm goal))
        (step-size 50)
        (remainingDist goal-distance))
    (while (> remainingDist 0)
      (format t ":reach-until-can-grasp loop remaining distance ~f~%" remainingDist)

      (open-hand arm)
      (send-angle-vector :time 500)
      ;; approach
      ;; (send *aero* :upper arm :move-end-pos (scale step-size goal-direction) :local :rotation-direction nil) 
      (send *aero* :upper arm :move-end-pos (scale step-size goal-direction) :local) 
      (send-angle-vector :time 100)
      (if (boundp '*irtviewer*)
          (send *irtviewer* :draw-objects))
      (close-hand arm)
      (send-angle-vector :time 1000)
      (if (boundp '*irtviewer*)
          (send *irtviewer* :draw-objects))
      (unix:sleep 2)
      (if (and (> (ri-hand-joint-angle arm) (aref hand-grasp-range 0)) ;; if hand isn't fully closed,
               (< (ri-hand-joint-angle arm) (aref hand-grasp-range 01)) ;; and it isn't widely opened, it means we've grapsed 
          (return-from reach-until-grasped t)
          )
      (setq remainingDist (- remainingDist step-size))
      )    
    )
  nil
  )

(defun find-grasp-target (arm grasp-vector scan-vector (&key hand-grasp-range #f(-40 0)))
    (let* ((scan-direction (normalize-vector scan-vector))
        (scan-distance (norm scan-vector))
        (step-size 50)
        (remainingDist scan-distance))
    (while (> remainingDist 0)
      (format t ":find-grasp-target loop remaining distance ~f~%" remainingDist)

      (if (reach-until-grasped arm grasp-vector :hand-grasp-range hand-grasp-range) ;; reach forward and try to grasp
          (return-from find-grasp-target t)
        (progn
          (send *aero* :upper arm :move-end-pos (v- grasp-vector) :local) 
          (send-angle-vector :time (* 1 *base-time*))
          ;; reach back
          (if (boundp '*irtviewer*)
              (send *irtviewer* :draw-objects))
          
          ;; scan forward
          (send *aero* :upper arm :move-end-pos (scale step-size scan-direction) :local) 
          (send-angle-vector :time (* 1 *base-time*))
          (if (boundp '*irtviewer*)
              (send *irtviewer* :draw-objects))
          )
        )
      (setq remainingDist (- remainingDist step-size))
      )   
    )
    nil
  )
    
(defun ri-hand-joint-angle (arm)
  (return-from ri-hand-joint-angle -10)
  (case arm
    (:rarm
      (aref (elt (send *ri* :actual-vector) 0) 7))
    (:larm
      (- (aref (elt (send *ri* :actual-vector) 0) 18))
      )
    )
  )
   
(defun close-hand (arm)
  (send *aero* :hand arm :joint-angle 50)
  )
   
(defun open-hand (arm)
  (send *aero* :hand arm :joint-angle -50)
  )
