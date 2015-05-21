;; (require :aero-upper "package://aero_ros_bridge/models/aero_upper.l")
;; (require :aero-lower "package://aero_ros_bridge/models/aero-lower.l")
(load "package://aero_ros_bridge/euslisp/aero-interface.l")
(require "package://aero_ros_bridge/euslisp/test-aero-lower-motion.l")


(defclass Walker
  :super object
  :slots (stride-length))
 )
(defmethod Walker
  (:init ()
    (setq stride-length 120)
    
    )
  (:step (leg)
    )
  (:walk (goal)
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
      )
    
    )

  ;; @param goal: the relative goal position to move leg to
  ;; @param leg: the leg to move
  (:step (goal leg)
    ;; (setq goal-coords (send (send *aero* :lower leg :end-coords :copy-worldcoords) :translate goal :world))
    (print (send *aero* :lower leg :end-coords :worldcoords))
    (setq stepSize 2) ;; tuneable param
    (setq stepHeight 100) ;; tuneable param
    (setq balance-rate-thre 1.5) ;;tuneable param
    (setq numSteps (/ (norm goal) stepSize))
    (setq i 1)
    (while (<= i numSteps)
      (setq vel (scale stepSize (normalize-vector goal))) ;; straight trajectory to goal
      (setq zvel (* (/ (* 8 stepHeight) numSteps) (- 0.5 (/ i numSteps)))) ;; add parabola vertical trajectory
      (setq vel (v+ vel (float-vector 0 0 zvel)))
      (if (> (send *aero* :rate-balance (remove leg `(:rarm :larm :rleg :lleg))) balance-rate-thre)
      ;; (if t
          (progn
            (send self :move-leg vel leg)
            (send *irtviewer* :draw-objects)
            (setq i (+ i 1))
            )
        (progn 
          (send self :shift-com-to-balance (remove leg `(:rarm :larm :rleg :lleg)))
          (format t ":step -- balance rate ~f less than threshold ~f, shift center of mass" (send *aero* :rate-balance (remove leg `(:rarm :larm :rleg :lleg))) balance-rate-thre)
          )
        )
      )
    ;; there might be some intergative error at this point after executing the trajectory.
    ;; adjust the foot by moving it to the goal position.
    ;; (send *aero-lower* leg :inverse-kinematics goal-coords :rotation-axis nil :thre 100)
    )

  ;; move center of mass (waist) towards the direction of the trianlge center formed by the supporting legs 
  (:shift-com-to-balance (&optional (legs-on-ground `(:rarm :rleg :lleg)))
    (let ((balance-center-direction (send self :get-balance-center-direction legs-on-ground)))
      (print balance-center-direction)
      (send *aero* :move-waist (scale 100 balance-center-direction))
      (send *irtviewer* :draw-objects)
      )
    )
  ;; returns a unit vector pointing in the direction from the waist center
  ;; to the centor of the triangle from by the three supporting legs
  (:get-balance-center-direction (&optional (legs-on-ground `(:rarm :rleg :lleg)))
    (let ((centroid (scale (/ 1.0 3)
                           (vplus (list (send *aero* :lower (car legs-on-ground) :end-coords :worldpos)
                                        (send *aero* :lower (cadr legs-on-ground) :end-coords :worldpos)
                                        (send *aero* :lower (caddr legs-on-ground) :end-coords :worldpos))))))
      ;; (send (make-coords :pos centroid) :draw-on :flush t)
      (setq balance-center-direction-3d (normalize-vector (v- centroid (send *aero* :lower :links :WAIST_LINK0 :worldpos))))
      (float-vector (aref balance-center-direction-3d 0) (aref balance-center-direction-3d 1) 0)
      )
    )
  ;; @param stride-length: how wide each step is
  ;; @param goal: the goal position relative to the robot
  (:walk (&optional (stride-length 120) (goal (float-vector 1000 0 0)))

    (setq debug nil)
    (setq gait-seq `(:larm :rleg :rarm :lleg)) ;; define gait sequence
    (setq goal-direction-vec (normalize-vector goal))
    (setq goal-pos (v+ goal (send *aero* :lower :links :WAIST_LINK0 :worldpos))) ;; goal coordinates
    (setq goal-pos-thre 500) ;; stop when we reach within goal-pos-thre from the goal
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
      (send self :step (scale stride-length goal-direction-vec) (car gait-seq))

      (if debug
          (progn
            (print "hit any key to move waist")
            (read-char)
            )
        )

      ;; shift waist by 1/4 stride-length towards goal direction
      (send *aero* :move-waist (v- next-waist-target-pos (send *aero* :lower :links :WAIST_LINK0 :worldpos)))
      (setq gait-seq `(,@(cdr gait-seq) ,(car gait-seq)))
      )
    )
  )

(send *aero-lower* :newcoords (make-coords :pos #f( 0 0 0 )))
(send *aero* :reset-terrain-pose)
(set-3legs-to-ground)
(setq walker (instance Walker :init))
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
