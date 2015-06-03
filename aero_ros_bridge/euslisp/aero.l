(require :aero-upper "package://aero_ros_bridge/models/aero_upper.l")
(require :aero-lower "package://aero_ros_bridge/models/aero-lower.l")
;; some methods of AeroUpperRobot-robot is in aero-upper-interface.l
(load "package://aero_ros_bridge/euslisp/aero-upper-strokes.l")
(load "package://aero_ros_bridge/euslisp/aero-upper-interface.l")
(load "package://aero_ros_bridge/euslisp/aero-lower-strokes.l")
(load "package://aero_ros_bridge/euslisp/aero-upper-lower-angles.l")
(load "package://aero_ros_bridge/euslisp/aero-go-pos.l")

(setq wheel-radius 44.63875)


(defclass AeroRobot-robot
  :super object
  :slots ())
(defmethod AeroRobot-robot


  ;;;;;;;;;;;;;;;;;; init ;;;;;;;;;;;;;;;;;;
  (:init ()
    (progn
      (if (not (boundp `*aero-upper*))
          (setq *aero-upper* (instance AeroUpperRobot-robot :init)))
      (if (not (boundp '*aero-lower*)) 
          (setq *aero-lower* (instance AeroLowerRobot-robot :init)))
      ;; move aero-upper to top location of aero-lower
      (if (boundp '*aero-lower*)
          (send *aero-upper* :newcoords
                (send (send *aero-lower* :torso :end-coords)
                      :worldcoords))
        )
      (send (send *aero-lower* :links :WAIST_LINK0) :assoc (send *aero-upper* :links :WAIST_LINK0)))
      (setq ((send *aero-upper* :head :neck-p) . max-angle) 40)

      ;; shin coords for kneeling
      (setq lshin-coords (send (send (make-cascoords :coords (send (send *aero-lower* :larm :end-coords) :copy-worldcoords)) :rotate (deg2rad -120) :y) :locate (float-vector 200 0 -40)))
      (send (send *aero-lower* :larm :end-coords :parent) :assoc lshin-coords)
      (setq rshin-coords (send (send (make-cascoords :coords (send (send *aero-lower* :rarm :end-coords) :copy-worldcoords)) :rotate (deg2rad -120) :y) :locate (float-vector 200 0 -40)))
      (send (send *aero-lower* :rarm :end-coords :parent) :assoc rshin-coords)
    )



  ;;;;;;;;;;;;;;;;;; for operating full body ;;;;;;;;;;;;;;;;;;
  (:angle-vector (&optional (angles nil))
    (if (eq angles nil)
	(list (send *aero-upper* :angle-vector) (send *aero-lower* :angle-vector))
      (progn
	(send *aero-upper* :angle-vector (elt angles 0))
	(send *aero-lower* :angle-vector (elt angles 1))
	angles))
    )
  (:stroke-vector ()
    (if (and (boundp `*aero-upper*) (boundp `*aero-lower*))
        (concatenate float-vector
                     (send *aero-upper* :stroke-vector) (send *aero-lower* :stroke-vector))
      )
    )
  (:translate (p &optional (wrt :local))
    (send *aero-lower* :translate p wrt)
    )
  (:locate (p &optional (wrt :local))
    (send *aero-lower* :locate p wrt)
    )
  (:rotate (theta axis &optional (wrt :local))
    (send *aero-lower* :rotate theta axis wrt)
    )
  (:upper (&rest args)
    (eval `(send *aero-upper* ,@args))
    )
  (:lower (&rest args)
    (eval `(send *aero-lower* ,@args))
    )
     
  (:move-waist
   (&optional (waist-velocity (float-vector 0 0 0))
	      &key (draw-objects t))
    (let ((original-end-coords-list) original-angle-vector
	  original-coords)
      (when (< (norm waist-velocity) 3)
	(warn ";; :move-waist error: waist-velocity is too small~%")
	(return-from :move-waist nil))
      ;;
      (setf original-angle-vector (send *aero-lower* :angle-vector))
      (setf original-coords (send *aero-lower* :copy-worldcoords))
      (dolist (link (list :rarm :larm :rleg :lleg))
        (push (send *aero-lower* link :end-coords :copy-worldcoords)
	      original-end-coords-list))
      (setf original-end-coords-list (nreverse original-end-coords-list))
      (send *aero-lower* :translate waist-velocity)
      (dolist (link (list :rarm :larm :rleg :lleg))
        (unless (send *aero-lower* link :inverse-kinematics
                       (car original-end-coords-list) :rotation-axis nil :stop 150)
          (warn ";; move-waist IK failed >> ~S~%" link)
          (send *aero-lower* :angle-vector original-angle-vector)
          (send *aero-lower* :newcoords original-coords)
          (return-from :move-waist nil)
          )
        (setf original-end-coords-list (cdr original-end-coords-list))
        )
      (unless (not (send self :self-collision-check))
        (warn ";; move-waist failed at self collision check~%")
        (send *aero-lower* :angle-vector original-angle-vector)
        (send *aero-lower* :newcoords original-coords)
        (return-from :move-waist nil)
        )
      (unless (eq (send self :cog-balance-check) t)
        (warn ";; move-waist failed: unbalacned after moving waist~%")
        (send *aero-lower* :angle-vector original-angle-vector)
        (send *aero-lower* :newcoords original-coords)
        (return-from :move-waist nil)
        )
      (if (and draw-objects (boundp '*irtviewer*))
          (send *irtviewer* :draw-objects))
      t)
    )

  (:rotate-waist
   (angle axis
	      &key (draw-objects t))
    (let ((original-end-coords-list) original-angle-vector
	  original-coords)
      (when (< (abs angle) (deg2rad 0.1))
	(warn ";; :rotate-waist error: angle is too small~%")
	(return-from :rotate-waist nil))
      ;;
      (setf original-angle-vector (send *aero-lower* :angle-vector))
      (setf original-coords (send *aero-lower* :links :WAIST_LINK0 :copy-coords))
      (dolist (link (list :rarm :larm :rleg :lleg))
        (push (send *aero-lower* link :end-coords :copy-worldcoords)
              original-end-coords-list))
      (setf original-end-coords-list (nreverse original-end-coords-list))
      (send *aero-lower* :links :WAIST_LINK0 :rotate angle axis)
      (dolist (link (list :rarm :larm :rleg :lleg))
        (unless (send *aero-lower* link :inverse-kinematics
                       (car original-end-coords-list) :rotation-axis nil)
          (warn ";; rotate-waist IK failed >> ~S~%" link)
          (send *aero-lower* :angle-vector original-angle-vector)
          (send *aero-lower* :links :WAIST_LINK0 :newcoords original-coords)
          (return-from :rotate-waist nil)
          )
        (setf original-end-coords-list (cdr original-end-coords-list))
        )
      (unless (not (send self :self-collision-check))
        (warn ";; rotate-waist failed at self collision check~%")
        (send *aero-lower* :angle-vector original-angle-vector)
        (send *aero-lower* :links :WAIST_LINK0 :newcoords original-coords)
        (return-from :rotate-waist nil)
        )
      (unless (eq (send self :cog-balance-check) t)
        (warn ";; rotate-waist failed: unbalacned after moving waist~%")
        (send *aero-lower* :angle-vector original-angle-vector)
        (send *aero-lower* :links :WAIST_LINK0 :newcoords original-coords)
        (return-from :move-waist nil)
        )

      (if (and draw-objects (boundp '*irtviewer*))
          (send *irtviewer* :draw-objects))
         t)
    )
  (:fix-leg-to-ground (&optional (legs '(:larm :rarm :lleg :rleg)))
                      (let ((leg-height 0))
      (dolist (x legs)
        (setq leg-height (+ (aref (send (send self :end-ground-coords x) :pos) 2) leg-height)))
      (setq leg-height (/ leg-height (length legs)))
      (send self :translate (float-vector 0 0 (- leg-height)))
      ))



  ;;;;;;;;;;;;;;;;;; for upper manipulation ;;;;;;;;;;;;;;;;;;
  (:hand (&rest args)
    (if (null args) (return-from :hand nil))
    (let ((key (car args))
	  (nargs (cdr args)))
      (unless (keywordp key)
	(return-from :hand nil))
      (case key
	    (:arms (progn (send *aero-upper* :hand-poses :larm nargs)
			  (send *aero-upper* :hand-poses :rarm nargs)))
	    (:larm (send *aero-upper* :hand-poses :larm nargs))
	    (:rarm (send *aero-upper* :hand-poses :rarm nargs))
	    (t nil)
	    )
      )
    )
  ;; hand-yaw's zero position is not parallel to ground at manip-pose
  ;; this function will help when you want to set angle values according to ground coords
  ;; please! only use this function at manip-pose
  (:hand-yaw-world (arm angle)
    (let (rot)
      (setq rot (car (rotation-angle
		      (send (send (send *aero-upper* :torso :end-coords) :copy-worldcoords)
			    :rot))))
      (if rot
	  (send self :rotate rot #f(0 0 1)))
      (send *aero-upper* arm :wrist-y :joint-angle 0)
      (send *aero-upper* arm :wrist-y :joint-angle
	    (+ (* (car (rotation-angle
			(send (send (send self :upper arm :end-coords)
				    :copy-worldcoords) :rot)))
		  (if (eq arm :larm) (/ 180 pi) (/ -180 pi)))
	       angle))
      (if rot
	  (send self :rotate (- rot) #f(0 0 1)))
      ))
  (:magic-close (arm &optional (x 0) (time 100) (real *real*))
    (if (< x 0) (return-from :magic-close (warn "first option must be positive~%")))
    (if (= (send self :hand arm :joint-angle) 50.0)
	(return-from :magic-close (warn "already closed~%")))
    (let (index-end angle-bits pos-bits forward-hand)
      (setq forward-hand (+ x 49.2))
      (setq index-end (send (send (send *aero-upper* arm :f-3p :child-link) :copy-worldcoords) :pos))
      (setq angle-bits (/ (- 50.0 (send self :hand arm :joint-angle)) time))
      (setq pos-bits (/ (float forward-hand) time))
      (dotimes (i time)
	(send self :hand arm :joint-angle (+ (send *aero-upper* :hand arm :joint-angle) angle-bits))
	(send *aero-upper* arm :move-end-pos
	      (if (> (aref index-end 0)
		     (aref (send (send (send *aero-upper* arm :f-3p :child-link)
				       :copy-worldcoords) :pos) 0))
		  (float-vector (- (aref index-end 0)
				   (aref (send (send (send *aero-upper* arm :f-3p :child-link)
						     :copy-worldcoords) :pos) 0)
				   pos-bits)
				0 0)
		(float-vector (- pos-bits) 0 0)) :world)
	(setq index-end (send (send (send *aero-upper* arm :f-3p :child-link) :copy-worldcoords) :pos))
    (if (boundp '*irtviewer*)
        (send *irtviewer* :draw-objects))
	(if (eq real t)
	    (progn (send *ri* :angle-vector (send self :stroke-vector) 20)
		   (unix::usleep (* 40 1000))))
	)
      ;; the robot is not going to completely close when grasping an object
      ;; therefore set goal to current angle
      (if (eq real t)
          (progn (send *aero* :hand arm :joint-angle
		       (if (eq arm :larm) (elt (car (send *ri* :actual-vector)) 18)
			 (- (elt (car (send *ri* :actual-vector)) 7))))
		 (send *ri* :angle-vector (send *aero* :stroke-vector) 100)
		 (unix::usleep (* 100 1000))
		 ))
      ))
  (:magic-open (arm &optional (x 0) (time 100) (real *real*))
    (if (> x 0) (return-from :magic-open (warn "first option must be negative~%")))
    (if (= (send self :hand arm :joint-angle) -50.0)
	(return-from :magic-open (warn "already open~%")))
    (let (index-end angle-bits pos-bits backward-hand)
      (setq backward-hand (- x 49.2))
      (setq index-end (send (send (send *aero-upper* arm :f-3p :child-link) :copy-worldcoords) :pos))
      (setq angle-bits (/ (- (send self :hand arm :joint-angle) -50.0) time))
      (setq pos-bits (/ (float backward-hand) time))
      (dotimes (i time)
	(send self :hand arm :joint-angle (- (send *aero-upper* :hand arm :joint-angle) angle-bits))
	(send *aero-upper* arm :move-end-pos
	      (if (> (aref index-end 0)
		     (aref (send (send (send *aero-upper* arm :f-3p :child-link) :copy-worldcoords) :pos) 0))
		  (float-vector (+ (- (aref (send (send (send *aero-upper* arm :f-3p :child-link)
							:copy-worldcoords) :pos) 0)
				      (aref index-end 0))
				   pos-bits)
				0 0)
		(float-vector pos-bits 0 0)) :world)
	(setq index-end (send (send (send *aero-upper* arm :f-3p :child-link) :copy-worldcoords) :pos))
    (if (boundp '*irtviewer*)
          (send *irtviewer* :draw-objects))
	(if (eq real t)
	    (progn (send *ri* :angle-vector (send self :stroke-vector) 20)
		   (unix::usleep (* 40 1000))))
	)
      ))



  ;;;;;;;;;;;;;;;;;; for lower manipulation ;;;;;;;;;;;;;;;;;;
  ;; x y components of target is relative to the shoulder/crotch joint.
  ;; If height-rel is true, z component of target is relative to limb endpoint height,
  ;; else it is absolute.
  (:kneel (target limb &key (height-rel t))
    (cond ((equal limb ':rarm)
           (progn
             (setq shin-coords rshin-coords)
             (setq shoulder-pos (send *aero-lower* :links :RARM_LINK1 :worldpos))))
           ((equal limb ':larm)
            (progn
              (setq shin-coords lshin-coords)
              (setq shoulder-pos (send *aero-lower* :links :LARM_LINK1 :worldpos))))
       )
    (setq target-worldpos (v+ (send self :project-on-ground shoulder-pos) target))
    (if height-rel
        (setq target-worldpos (v+ target-worldpos (float-vector 0 0 (elt (send (send self :end-ground-coords limb) :worldpos) 2)))))
    (send *aero-lower* limb :inverse-kinematics  (make-coords :pos target-worldpos) :move-target shin-coords :debug-view t :rotation-axis :z :translation-axis :x)
    )

  ;;;;;;;;;;;;;;;;;; for evaluating ;;;;;;;;;;;;;;;;;;
  (:self-collision-check ()
    (and (send *aero-upper* :collision-check-all)
	 (send *aero-lower* :self-collision-check))
    )
  (:cog-balance-check (&optional (legs-on-grounds `(:larm :rarm :rleg :lleg)))
    (let (a b c p A1 B1 c-eval p-eval (tmp-legs))
      (setq tmp-legs legs-on-grounds)
      (setq p (send self :get-com))
      ;; check com
      (dolist (x legs-on-grounds)
	(setq c (send (send *aero-lower* (car tmp-legs) :end-coords :copy-worldcoords) :pos))
	(setq a (send (send *aero-lower* (cadr tmp-legs) :end-coords :copy-worldcoords) :pos))
	(setq b (send (send *aero-lower* (caddr tmp-legs) :end-coords :copy-worldcoords) :pos))
	(setq A1 (- (elt b 1) (elt a 1)))
	(setq B1 (- (elt b 0) (elt a 0)))
	(setq c-eval (if (> (- (* A1 (- (elt c 0) (elt a 0)))
			       (* B1 (- (elt c 1) (elt a 1)))) 0) t nil))
	(setq p-eval (if (> (- (* A1 (- (elt p 0) (elt a 0)))
			       (* B1 (- (elt p 1) (elt a 1)))) 0) t nil))
	(if (eq c-eval p-eval) nil (return-from :cog-balance-check p))
	(setq tmp-legs `(,@(cdr tmp-legs) ,(car tmp-legs)))
	)
      t)
    )
  (:rate-balance (&optional (legs-on-grounds `(:rarm :rleg :lleg)))
    (let ((com-on-ground (send self :project-on-ground (send self :get-com)))
	  (get-3foot-center #f(0 0 0))
	  (tmp-legs legs-on-grounds)
	  edge-length
	  (edge-lengths `())
	  (s 0)
	  (area 1)
	  inner-radius
	  )
      (if (eq (send self :cog-balance-check legs-on-grounds) t)
	  (progn (dolist (link legs-on-grounds)
		   (setq foot-pos (send self :project-on-ground
					(send (send *aero-lower* (car tmp-legs) :end-coords :copy-worldcoords)
					      :pos)))
		   (setq edge-length (norm (v- foot-pos
					       (send self :project-on-ground
						     (send (send *aero-lower* (cadr tmp-legs)
								 :end-coords :copy-worldcoords)
							   :pos)))))
		   (setq get-3foot-center (v+ get-3foot-center foot-pos))
		   (setq edge-lengths (push edge-length edge-lengths)) 
		   (setq tmp-legs `(,@(cdr tmp-legs) ,(car tmp-legs)))
		   )
		 (dolist (x edge-lengths)
		   (setq s (+ s x)))
		 (setq get-3foot-center (scale (/ 1.0 3) get-3foot-center))
		 (send (make-coords :pos get-3foot-center) :draw-on :flush t :size 500)
		 (setq s (/ s 2))
		 (dolist (x edge-lengths)
		   (setq area (* (- s x) area)))
		 (setq area (sqrt (* s area)))
		 (setq inner-radius (/ area s))
		 (/ inner-radius (norm (v- get-3foot-center com-on-ground)))
		 )
	0.0)
      )
    )



  ;;;;;;;;;;;;;;;;;; for returning values ;;;;;;;;;;;;;;;;;;
  ;; calculate center of mass of the whole robot using center of mass of aero upper and aero lower
  (:get-com ()
    (scale (/ 1 (+ (send *aero-upper* :weight) (send *aero-lower* :weight)))
           (v+ (scale (send *aero-upper* :weight) (send *aero-upper* :centroid))
               (scale (send *aero-lower* :weight) (send *aero-lower* :centroid))))
    )
  ;; helper function for projecting a vector onto ground plane
  (:project-on-ground (in_coords)
    (float-vector (aref in_coords 0) (aref in_coords 1) 0)
    )
  ;; The end coords are at the wheels' axes
  ;; The end ground coords give the point that is touching the ground
  (:end-ground-coords (limb)
    (send (send (send self :lower limb :end-coords) :copy-worldcoords) :translate (float-vector 0 0 (- wheel-radius)) :world)
    ;; (eval `(send self :lower limb :end-coords))
    )



  ;;;;;;;;;;;;;;;;;; for viewing ;;;;;;;;;;;;;;;;;;
  (:draw-on ()
    ;; executing (objects *aero*) is not going to work. Use this function instead to visualize aero in irtviewer
    (objects *aero-upper*)
    (objects *aero-lower*)
    (if (boundp '*irtviewer*)
        (send *irtviewer* :draw-objects))
    )
  ;; draw center of mass projected onto ground plane
  (:draw-centroid-on-ground ()
    (let ((com-on-ground (make-coords :pos (send self :project-on-ground (send self :get-com))))
          )
      (send com-on-ground :draw-on :flush t)
      com-on-ground)  
    )
  (:draw-upper-centroid-on-ground ()
    (let ((com-on-ground (make-coords :pos (send self :project-on-ground (send *aero-upper* :centroid))))
          )
      (send com-on-ground :draw-on :flush t)
      com-on-ground)  
    )
  (:draw-lower-centroid-on-ground ()
    (let ((com-on-ground (make-coords :pos (send self :project-on-ground (send *aero-lower* :centroid))))
          )
      (send com-on-ground :draw-on :flush t)
      com-on-ground)  
    )



  ;;;;;;;;;;;;;;;;;; poses ;;;;;;;;;;;;;;;;;;  
  (:reset-pose ()
    (send *aero-upper* :reset-pose)
    (send *aero-lower* :reset-pose)
    (send *aero-lower* :links :WAIST_LINK0 :newcoords
          (make-coords :pos (send *aero-lower* :links :WAIST_LINK0 :pos) :rpy #f(0 0 0)))
    (send self :fix-leg-to-ground)
    )
  (:reset-manip-pose ()
    (send *aero-upper* :reset-manip-pose)
    (send *aero-lower* :reset-manip-pose)
    (send *aero-lower* :links :WAIST_LINK0 :newcoords
          (make-coords :pos (send *aero-lower* :links :WAIST_LINK0 :pos) :rpy #f(0 0 0)))
    (send self :fix-leg-to-ground)
    )
  
  (:reset-terrain-pose ()
    (send *aero-upper* :reset-terrain-pose)
    (send *aero-lower* :reset-terrain-pose)
    (send *aero-lower* :links :WAIST_LINK0 :newcoords
          (make-coords :pos (send *aero-lower* :links :WAIST_LINK0 :pos) :rpy #f(0 0 0)))
    (send self :fix-leg-to-ground)
    )

  ;; waist posiion slightly higher than terrain-pose
  (:reset-terrain-pose2 ()
    (send self :angle-vector (list #f(0.0 -15.0 0.0 -90.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 15.0 0.0 -90.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0) #f(35.0 30.0 80.0 -35.0 30.0 80.0 -35.0 30.0 80.0 35.0 30.0 80.0 0.0)))
    (send *aero-lower* :links :WAIST_LINK0 :newcoords
          (make-coords :pos (send *aero-lower* :links :WAIST_LINK0 :pos) :rpy #f(0 0 0)))
    (send self :fix-leg-to-ground)
    )

  (:reset-climbing-pose ()
    (send self :angle-vector (list #f(0.0 -15.0 0.0 -90.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 15.0 0.0 -90.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0) #f(0.0 40.0 80.0 0.0 40.0 80.0 0.0 40.0 80.0 0.0 40.0 80.0 0.0)))
    (send *aero-lower* :links :WAIST_LINK0 :newcoords
          (make-coords :pos (send *aero-lower* :links :WAIST_LINK0 :pos) :rpy #f(0 0 0)))
    (send self :fix-leg-to-ground)
    )

  (:reset-climbing-pose2 ()
    (send self :angle-vector (list #f(0.0 -15.0 0.0 -90.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 15.0 0.0 -90.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0) #f(73.7016 40.3432 78.5655 -0.1388 36.0831 87.7224 -73.6487 40.3196 78.6764 -0.1388 36.0831 87.7224 0.0)))
    (send *aero-lower* :links :WAIST_LINK0 :newcoords
          (make-coords :pos (send *aero-lower* :links :WAIST_LINK0 :pos) :rpy #f(0 0 0)))
    (send self :fix-leg-to-ground)
    )

  ;; save the current pose of the robot
  (:save-pose ()
    (setq saved-pose (send self :angle-vector))
    )
  ;; load the previously saved pose of the robot
  (:load-pose ()
    (send self :angle-vector saved-pose)
    )
  ;; save the current pose and global position of the robot
  (:save-state ()
    (send self :save-pose)
    (setq saved-robot-coords (send *aero-lower* :copy-worldcoords))
    (setq saved-waist-coords (send *aero-lower* :links :WAIST_LINK0 :copy-coords))
    )
  ;; load the previously saved pose and global position of the robot
  (:load-state ()
    (send self :load-pose)
    (send *aero-lower* :newcoords (send saved-robot-coords :copy-coords))
    (send *aero-lower* :links :WAIST_LINK0 :newcoords saved-waist-coords)
    (if (boundp '*irtviewer*)
        (send *irtviewer* :draw-objects))
    )

  (:match-ri ()
    (send self :angle-vector (send *ri* :actual-vector))
    (if (boundp '*irtviewer*)
        (send *irtviewer* :draw-objects))
    )
 
  );; end of defmethod AeroRobot-robot



(provide :aero "package://aero_ros_bridge/euslisp/aero.l")
