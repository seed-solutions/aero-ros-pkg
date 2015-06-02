;; (require :aero-upper "package://aero_ros_bridge/models/aero_upper.l")
;; (require :aero-lower "package://aero_ros_bridge/models/aero-lower.l")
(load "package://aero_ros_bridge/euslisp/aero-interface.l")


(defclass Walker
  :super object
  :slots (def_balance_rate_thre def_step_height def_step_square_height def_stride_length
           def_walk_goal))
 )
(defmethod Walker
  (:init ()
    (send self :set-val 'def_balance_rate_thre 5)
    (send self :set-val 'def_step_height 100)
    (send self :set-val 'def_step_square_height 10)
    (send self :set-val 'def_stride_length 120)
    (send self :set-val 'def_walk_goal #f(500 0 0))
    )
  ;; @param goal: the coords of the final goal where the robot wants to go
  ;; @param stepping-leg: which leg is stepping
  ;; returns a safe location to place stepping-leg
  (:get-next-foothold (goal stepping-leg)
    (let ((goal-vec (normalize-vector (v- goal (send self :get-current-loc)))))
      (v+ (send (send *aero* :end-ground-coords stepping-leg) :pos) (scale stride-length goal-vec))
      )
    )
  (:show-next-foothold (goal stepping-leg)
    (send (make-coords :pos (send self :get-next-foothold goal stepping-leg)) :draw-on :flush t)
    )

  (:get-current-loc ()
    (float-vector (aref (send (send *aero* :lower :worldcoords) :pos) 0)
                  (aref (send (send *aero* :lower :worldcoords) :pos) 1)
                  0)
    )
  ;; @param velocity: velocity to move the leg
  ;; @param leg: which leg to move
  (:move-leg (velocity leg)
    (setq current-leg-end-coords (send *aero* :lower leg :end-coords :copy-worldcoords))
    (unless
        (send *aero-lower* leg :inverse-kinematics (send current-leg-end-coords :translate velocity :world) :rotation-axis nil :thre 0.1)
      (warn ";; move-leg IK failed >> ~S" leg)
      (return-from :move-leg nil)
      )
    t
    )

  ;; @param goal: the relative goal position to move leg to
  ;; @param leg: the leg to move
  (:step (goal leg &key (balance-rate-thre (send self :get-val `def_balance_rate_thre)) (step-height (send self :get-val `def_step_height)))
    (setq goal-coords (send (send *aero* :lower leg :end-coords :copy-worldcoords) :translate goal :world))
    (print (send *aero* :lower leg :end-coords :worldcoords))
    (setq stepSize 10) ;; tuneable param
    (setq numSteps (/ (norm goal) stepSize))
    (setq i 1)
    (while (<= i numSteps)
      (setq vel (scale stepSize (normalize-vector goal))) ;; straight trajectory to goal
      (setq zvel (* (/ (* 8 step-height) numSteps) (- 0.5 (/ i numSteps)))) ;; add parabola vertical trajectory
      (setq vel (v+ vel (float-vector 0 0 zvel)))
      ;; (print (send *aero* :rate-balance (remove leg `(:rarm :larm :rleg :lleg))))
      (if (> (send *aero* :rate-balance (remove leg `(:rarm :larm :rleg :lleg))) balance-rate-thre)
      ;; (if t
          (progn
            (unless (send self :move-leg vel leg)
              (print ":step failed move-leg")
              (return-from :step nil)
              )

            (if (boundp '*irtviewer*)
                (send *irtviewer* :draw-objects))
            (setq i (+ i 1))
            )
        (progn 
          (unless (send self :shift-com-to-balance (remove leg `(:rarm :larm :rleg :lleg)) balance-rate-thre)
            (print ":step -- shift-com-to-balance failed")
            (return-from :step nil))
          )
        )
      )
    ;; there might be some intergative error at this point after executing the trajectory.
    ;; need to adjust the foot by moving it to the goal position.
    (send *aero-lower* leg :inverse-kinematics goal-coords :rotation-axis nil :thre 100)
    t
    )

  (:step-square (goal leg &key (balance-rate-thre (send self :get-val `def_balance_rate_thre)) (step-height (send self :get-val `def_step_height)))
    ;; up stroke
    (unless (send self :step (float-vector 0 0 (+ (elt goal 2) step-height)) leg :balance-rate-thre balance-rate-thre :step-height 0)
      (print ":step-square failed at up stroke")
      (return-from :step-square nil)
      )
    ;; forward stroke
    (unless (send self :step (float-vector (elt goal 0) (elt goal 1) 0) leg :balance-rate-thre balance-rate-thre :step-height 0)
      (print ":step-square failed at forward stroke")
      (return-from :step-square nil)
      )
    ;; down stroke
    (unless (send self :step (float-vector 0 0 (- step-height)) leg :balance-rate-thre balance-rate-thre :step-height 0)
      (print ":step-square failed at down stroke")
      (return-from :step-square nil)
      )
    t
    )
         
  ;; move center of mass (waist) towards the direction of the trianlge center formed by the supporting legs 
  (:shift-com-to-balance (&optional (legs-on-ground `(:rarm :rleg :lleg)) (required-balance-rate 3) &key (step-size 0.5))
    (setq count 0)
    (while  (< (send *aero* :rate-balance legs-on-ground) required-balance-rate)
      (setq balance-center-direction (send self :get-balance-center-direction legs-on-ground))
      (format t ":shift-com-to-balance ~a~%" legs-on-ground)
      (format t "  balance rate ~f less than threshold ~f, shift center of mass~%" (send *aero* :rate-balance legs-on-ground) required-balance-rate)
      (send *aero* :move-waist (scale step-size balance-center-direction))
      (format t ":shift-com-to-balance -- balance rate after shifting center of mass ~f~%" (send *aero* :rate-balance legs-on-ground) required-balance-rate)
      (if (boundp '*irtviewer*)
          (send *irtviewer* :draw-objects))
      (if (> count 20)
          (progn
            (print ":shift-com-to-balance loop reached max iteration")
            (return-from :shift-com-to-balance nil)
            )
        )
      (setq count (+ count 1))
    )
    t
    )

  ;; Move center of mass on top of the trianlge center formed by the supporting legs.
  ;; NOTE: this actually simply moves the waist on top of the triangle center for now.
  (:shift-com-to-support-center (&optional (legs-on-ground `(:rarm :rleg :lleg)))
    (let ((balance-center-direction (send self :get-balance-center-direction legs-on-ground)))
      (print balance-center-direction)
      (send *aero* :move-waist balance-center-direction)
      (if (boundp '*irtviewer*)
          (send *irtviewer* :draw-objects))
      )
    )

  ;; Move center of mass (waist) on top of the quadrilateral center formed by the supporting legs.
  ;; NOTE: this actually simply moves the waist on top of the triangle center for now.
  (:shift-com-to-center ()
    (let ((leg-quadrilateral-center-direction (send self :get-leg-quadrilateral-center-direction)))
      (print leg-quadrilateral-center-direction)
      (send *aero* :move-waist leg-quadrilateral-center-direction)
      (if (boundp '*irtviewer*)
          (send *irtviewer* :draw-objects))
      )
    )


  ;; returns a 2d vector parallel to the ground plane pointing in the direction from the waist center
  ;; to the centor of the triangle from by the three supporting legs
  (:get-balance-center-direction (&optional (legs-on-ground `(:rarm :rleg :lleg)))
    (let ((centroid (scale (/ 1.0 3)
                           (vplus (list (send *aero* :lower (car legs-on-ground) :end-coords :worldpos)
                                        (send *aero* :lower (cadr legs-on-ground) :end-coords :worldpos)
                                        (send *aero* :lower (caddr legs-on-ground) :end-coords :worldpos))))))
      ;; (send (make-coords :pos centroid) :draw-on :flush t)
      ;; (setq balance-center-direction-3d (normalize-vector (v- centroid (send *aero* :lower :links :WAIST_LINK0 :worldpos))))
      (setq balance-center-direction-3d (v- centroid (send *aero* :get-com)))
      (float-vector (aref balance-center-direction-3d 0) (aref balance-center-direction-3d 1) 0)
      )
    )

  (:get-leg-quadrilateral-center-direction ()
    (let ((centroid (scale (/ 1.0 4)
                           (vplus (list (send *aero* :lower :rarm :end-coords :worldpos)
                                        (send *aero* :lower :larm :end-coords :worldpos)
                                        (send *aero* :lower :rleg :end-coords :worldpos)
                                        (send *aero* :lower :lleg :end-coords :worldpos))))))
      ;; (send (make-coords :pos centroid) :draw-on :flush t)
      ;; (setq balance-center-direction-3d (normalize-vector (v- centroid (send *aero* :lower :links :WAIST_LINK0 :worldpos))))
      (setq leg-quadrilateral-center-direction-3d (v- centroid (send *aero* :get-com)))
      (float-vector (aref leg-quadrilateral-center-direction-3d 0) (aref leg-quadrilateral-center-direction-3d 1) 0)
      )
    )

  ;; @param stride-length: how wide each step is
  ;; @param goal: the goal position relative to the robot
  (:walk (&key (stride-length (send self :get-val `def_stride_length)) (goal (send self :get-val `def_walk_goal)) (balance-rate-thre (send self :get-val `def_balance_rate_thre)) (step-height (send self :get-val `def_step_height)))

    (setq debug nil)
    (setq gait-seq `(:larm :rleg :rarm :lleg)) ;; define gait sequence
    (setq goal-direction-vec (normalize-vector goal))
    (setq goal-pos (v+ goal (send *aero* :lower :links :WAIST_LINK0 :worldpos))) ;; goal coordinates
    (setq goal-pos-thre 100) ;; stop when we reach within goal-pos-thre from the goal
    (while (> (distance (send *aero* :lower :links :WAIST_LINK0 :worldpos) goal-pos) goal-pos-thre)
      ;; Each time we step a leg, we want to move the waist in goal direction by 1/4 stride-length
      ;; Since when we step a leg, the waist might be moved for balancing, calculate the desired
      ;; waist target pos now first.
      (setq waist-start-pos (send *aero* :lower :links :WAIST_LINK0 :worldpos))
      (setq next-waist-target-pos (v+ waist-start-pos (scale (* 0.25 stride-length) goal-direction-vec)))

      (if debug
          (progn
            (print "hit any key to step")
            (print (car gait-seq))
            (read-char)
            )
        )

      ;; step the next leg forward
      (unless (send self :step (scale stride-length goal-direction-vec) (car gait-seq) :balance-rate-thre balance-rate-thre :step-height step-height)
        (format t ":walk -- failed to step >> ~s~%" (car gait-seq))
        (return-from :walk nil)
        )

      (if debug
          (progn
            (print "hit any key to move waist")
            (read-char)
            )
        )

      ;; shift waist by 1/4 stride-length towards goal direction
      (unless (send *aero* :move-waist (v- next-waist-target-pos (send *aero* :lower :links :WAIST_LINK0 :worldpos)))
        (format t ":walk -- failed to move-waist forward~%")
        (return-from :walk nil)
        )
      
      (setq gait-seq `(,@(cdr gait-seq) ,(car gait-seq)))
      )
    (format t "goal reached within ~f~%" goal-pos-thre)
    )
  )

(if (not (boundp '*walker*))
    (setq *walker* (instance Walker :init)))
;; (send walker :show-next-foothold goal :larm)
;; (setq foothold (send walker :get-next-foothold goal :larm))
;; (setq vel 2)
;; (send walker :step #f(100 0 0) :larm)
;; (send *irtviewer* :draw-objects)
;; (send walker :move-waist goal)
;; (send *irtviewer* :draw-objects)
;; (send walker :get-balance-center-direction)

;; (setq legs-on-ground `(:rarm :lleg :rleg))
;; (send (send *aero* :lower (car legs-on-ground) :end-coords :worldpos) :draw-on :flush t)
;; (send (send *aero* :lower (cadr legs-on-ground) :end-coords :worldpos) :draw-on :flush t)
;; (send (send *aero* :lower (caddr legs-on-ground) :end-coords :worldpos) :draw-on :flush t)
