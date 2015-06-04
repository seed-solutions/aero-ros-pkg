(defun ready-pose-valve ()
  (send *aero* :reset-manip-pose)
  (send *aero* :upper :rarm :move-end-pos #f(0 -100 100))
  (send *aero* :upper :rarm :wrist-y :joint-angle -30)
  (send *aero* :hand :rarm :joint-angle -50)
  (if (boundp '*irtviewer*)
      (send *irtviewer* :draw-objects))
  )

(defun grasp-and-rotate ()
  (setq rotate-vector #f(0 0 30))
  (setq start-angle-vector (send *aero* :angle-vector))
  (setq grasp-vector #f(100 100 0))
  (setq arm :rarm)
  (if (find-grasp-target arm grasp-vector #f(100 -100 0)) ;; if we grasp the valve successfully 
      (progn
        ;; rotate up 
        (print "grasp-and-rotate -- rotate up")
        (send *aero* :upper arm :move-end-pos rotate-vector :local)
        (send-angle-vector :time (* 1 *base-time*))
        (if (boundp '*irtviewer*)
            (send *irtviewer* :draw-objects))
        (unix:sleep 1)
        
        ;; opne hand
        (open-hand :rarm)
        (print "grasp-and-rotate -- open hand")
        (send-angle-vector :time (* 1 *base-time*))
        (if (boundp '*irtviewer*)
            (send *irtviewer* :draw-objects))
        (unix:sleep 1)
        
        ;; move hand back
        (print "grasp-and-rotate -- move hand back")
        (send *aero* :upper arm :move-end-pos (scale -50 (normalize-vector grasp-vector)) :local)
        (send-angle-vector :time (* 1 *base-time*))
        (if (boundp '*irtviewer*)
            (send *irtviewer* :draw-objects))
        (unix:sleep 1)
        
        ;; move back to start position
        (print "grasp-and-rotate -- move back to start position ")
        (send *aero* :angle-vector start-angle-vector) 
        (send-angle-vector :time (* 1 *base-time*))
        )
    )
  )
