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
          (setq *aero-upper* (instance AeroUpperRobot-robot :init))
	)
      (if (not (boundp '*aero-lower*)) 
          (setq *aero-lower* (instance AeroLowerRobot-robot :init))
	)
      ;; move aero-upper to top location of aero-lower
      (if (boundp '*aero-lower*)
          (send *aero-upper* :newcoords
                (send (send *aero-lower* :torso :end-coords)
                      :worldcoords))
        )
      (send (send *aero-lower* :links :WAIST_LINK0)
	    :assoc (send *aero-upper* :links :WAIST_LINK0)))
      (setq ((send *aero-upper* :head :neck-p) . max-angle) 40)
      (setq ((send *aero-lower* :rarm :shoulder-y) . max-angle) 0)
      (setq ((send *aero-lower* :larm :shoulder-y) . min-angle) 0)
    )



  ;;;;;;;;;;;;;;;;;; for operating full body ;;;;;;;;;;;;;;;;;;
  (:angle-vector (&optional (angles nil))
    (if (eq angles nil)
	(return-from :angle-vector
		     (list (send *aero-upper* :angle-vector) (send *aero-lower* :angle-vector)))
      )
    (send *aero-upper* :angle-vector (elt angles 0))
    (send *aero-lower* :angle-vector (elt angles 1))
    angles)
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
  (:upper (&rest args) ;; will be deprecated
    (eval `(send *aero-upper* ,@args))
    )
  (:lower (&rest args) ;; will be deprecated
    (eval `(send *aero-lower* ,@args))
    )
  (:move-waist (&optional (waist-velocity (float-vector 0 0 0)) &key (draw-objects t))
    (let ((original-end-coords-list)
	  original-angle-vector
	  original-coords
	  )
      (when (< (norm waist-velocity) 3)
        (unless *real* (warn ";; :move-waist error: waist-velocity is too small~%") )
	(return-from :move-waist nil)
	)
      (setf original-angle-vector (send *aero-lower* :angle-vector))
      (setf original-coords (send *aero-lower* :copy-worldcoords))
      (dolist (link (list :rarm :larm :rleg :lleg))
        (push (send *aero-lower* link :end-coords :copy-worldcoords)
	      original-end-coords-list)
	)
      (setf original-end-coords-list (nreverse original-end-coords-list))
      (send *aero-lower* :translate waist-velocity)
      (dolist (link (list :rarm :larm :rleg :lleg))
        (unless (send *aero-lower* link :inverse-kinematics
		      (car original-end-coords-list) :rotation-axis nil :stop 150)
          (unless *real* (warn ";; move-waist IK failed >> ~S~%" link))
          (send *aero-lower* :angle-vector original-angle-vector)
          (send *aero-lower* :newcoords original-coords)
          (return-from :move-waist nil)
          )
        (setf original-end-coords-list (cdr original-end-coords-list))
        )
      (unless (not (send self :self-collision-check))
        (unless *real* (warn ";; move-waist failed at self collision check~%"))
        (send *aero-lower* :angle-vector original-angle-vector)
        (send *aero-lower* :newcoords original-coords)
        (return-from :move-waist nil)
        )
      (unless (eq (send self :cog-balance-check) t)
        (unless *real* (warn ";; move-waist failed: unbalacned after moving waist~%"))
        (send *aero-lower* :angle-vector original-angle-vector)
        (send *aero-lower* :newcoords original-coords)
        (return-from :move-waist nil)
        )
      (if (and draw-objects (boundp '*irtviewer*))
          (send *irtviewer* :draw-objects)
	)
      t))
  (:rotate-waist (angle axis &key (draw-objects t))
    (let ((original-end-coords-list)
	  original-angle-vector
	  original-coords
	  )
      (when (< (abs angle) (deg2rad 0.1))
        (unless *real* (warn ";; :rotate-waist error: angle is too small~%"))
	(return-from :rotate-waist nil)
	)
      (setf original-angle-vector (send *aero-lower* :angle-vector))
      (setf original-coords (send *aero-lower* :links :WAIST_LINK0 :copy-coords))
      (dolist (link (list :rarm :larm :rleg :lleg))
        (push (send *aero-lower* link :end-coords :copy-worldcoords)
              original-end-coords-list)
	)
      (setf original-end-coords-list (nreverse original-end-coords-list))
      (send *aero-lower* :links :WAIST_LINK0 :rotate angle axis)
      (dolist (link (list :rarm :larm :rleg :lleg))
        (unless (send *aero-lower* link :inverse-kinematics
		      (car original-end-coords-list) :rotation-axis nil)
          (unless *real* (warn ";; rotate-waist IK failed >> ~S~%" link))
          (send *aero-lower* :angle-vector original-angle-vector)
          (send *aero-lower* :links :WAIST_LINK0 :newcoords original-coords)
          (return-from :rotate-waist nil)
          )
        (setf original-end-coords-list (cdr original-end-coords-list))
        )
      (unless (not (send self :self-collision-check))
        (unless *real* (warn ";; rotate-waist failed at self collision check~%"))
        (send *aero-lower* :angle-vector original-angle-vector)
        (send *aero-lower* :links :WAIST_LINK0 :newcoords original-coords)
        (return-from :rotate-waist nil)
        )
      (unless (eq (send self :cog-balance-check) t)
        (unless *real* (warn ";; rotate-waist failed: unbalacned after moving waist~%"))
        (send *aero-lower* :angle-vector original-angle-vector)
        (send *aero-lower* :links :WAIST_LINK0 :newcoords original-coords)
        (return-from :move-waist nil)
        )
      (if (and draw-objects (boundp '*irtviewer*))
          (send *irtviewer* :draw-objects)
	)
      t))
  (:fix-leg-to-ground (&optional (legs '(:larm :rarm :lleg :rleg)))
    (let ((leg-height 0)
	  )
      (dolist (x legs)
        (setq leg-height (+ (aref (send (send self :end-ground-coords x) :pos) 2) leg-height))
	)
      (setq leg-height (/ leg-height (length legs)))
      (send self :translate (float-vector 0 0 (- leg-height)))
      ))



  ;;;;;;;;;;;;;;;;;; for upper manipulation ;;;;;;;;;;;;;;;;;;
  (:hand (&rest args)
    (if (null args) (return-from :hand nil))
    (let ((key (car args))
	  (nargs (cdr args))
	  )
      (unless (keywordp key) (return-from :hand nil))
      (case key
	    (:arms
	     (send *aero-upper* :hand-poses :larm nargs)
	     (send *aero-upper* :hand-poses :rarm nargs)
	     )
	    (:larm
	     (send *aero-upper* :hand-poses :larm nargs))
	    (:rarm
	     (send *aero-upper* :hand-poses :rarm nargs))
	    (t
	     nil)
	    )
      ))
  (:grasp (arm &optional (time 500))
    (send self :hand arm :close-pose)
    (send *ri* :angle-vector (send self :stroke-vector) time)
    (unix::usleep (* (+ time 1000) 1000)) ;; sleep an extra 1 sec to get proper value
    (send self :hand arm :joint-angle
	  (if (eq arm :larm) (elt (car (send *ri* :actual-vector)) 18)
	    (- (elt (car (send *ri* :actual-vector)) 7))))
    (send *ri* :angle-vector (send self :stroke-vector) 100)
    (unix::usleep(* 100 1000))
    )
  (:ungrasp (arm &optional (time 100))
    (send self :hand arm :open-pose)
    (send *ri* :angle-vector (send self :stroke-vector) time)
    (unix::usleep(* time 1000))
    )
  ;; do not pick faster than 1 sec
  (:pick (arm &optional (time 1000))
    (let ((half-time (round (* time 0.5)))
	  (toward (m* (send (send *aero-upper* arm :end-coords :copy-worldcoords) :rot)
		      (rotation-matrix (deg2rad (if (eq arm :larm) -45 45)) #f(0 0 1))
		      (inverse-matrix
		       (send (send *aero-upper* arm :end-coords :copy-worldcoords) :rot))))
	  )
      (send self :hand arm :joint-angle -20) ;; tuned value -20
      (send *aero-upper* arm :move-end-pos (transform toward #f(15 0 0)) :world) ;; tuned value 15
      (send *ri* :angle-vector (send self :stroke-vector) half-time)
      (unix::usleep (* (+ half-time 10) 1000))
      (send self :grasp arm half-time)
      ))
  ;; do not unpick faster than 1 sec
  (:unpick (arm &optional (time 1000))
    (let ((toward (m* (send (send *aero-upper* arm :end-coords :copy-worldcoords) :rot)
		      (rotation-matrix (deg2rad (if (eq arm :larm) -45 45)) #f(0 0 1))
		      (inverse-matrix
		       (send (send *aero-upper* arm :end-coords :copy-worldcoords) :rot))))
	  )
      (send *aero-upper* arm :move-end-pos (transform toward #f(-15 0 0)) :world) ;; tuned value 15
      (send self :ungrasp arm time)
      ))
  ;; do not grasp-hard faster than 1 sec
  (:grasp-hard (arm &optional (time 1000) (x 0))
    (if (< x 0) (return-from :grasp-hard (warn "first option must be positive~%")) )
    (if (= (send self :hand arm :joint-angle) 50.0)
	(return-from :grasp-hard (warn "already closed~%"))
      )
    (let ((grasp-angle 1.0)
	  )
      (if (> (send self :hand arm :joint-angle) 0)
	  (setq grasp-angle (* (- 50 (send self :hand arm :joint-angle)) 0.02))
	)
      (send *aero-upper* arm :move-end-pos (float-vector (+ x (* 49.2 grasp-angle)) 0 0) :local
	    :revert-if-fail nil)
      (send self :grasp :larm time)
      ))
  ;; do not ungrasp-hard faster than 1 sec
  (:ungrasp-hard (arm &optional (time 1000) (x 0))
    (if (> x 0) (return-from :ungrasp-hard (warn "first option must be negative~%")) )
    (if (= (send self :hand arm :joint-angle) -50.0)
	(return-from :ungrasp-hard (warn "already open~%"))
      )
    (let ((grasp-angle 1.0)
	  )
      (if (> (send self :hand arm :joint-angle) 0)
	  (setq grasp-angle (* (send *aero* :hand arm :joint-angle) 0.02))
	)
      (send *aero-upper* arm :move-end-pos (float-vector (- x (* 49.2 grasp-angle)) 0 0) :local
	    :revert-if-fail nil)
      (send self :ungrasp :larm time)
      ))



  ;;;;;;;;;;;;;;;;;; for lower manipulation ;;;;;;;;;;;;;;;;;;



  ;;;;;;;;;;;;;;;;;; for evaluating ;;;;;;;;;;;;;;;;;;
  (:self-collision-check ()
    (and (send *aero-upper* :collision-check-all)
	 (send *aero-lower* :self-collision-check))
    )
  (:cog-balance-check (&optional (legs-on-grounds `(:larm :rarm :rleg :lleg)))
    (let (a b c p A1 B1 c-eval p-eval (tmp-legs)
	  )
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
      t))
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
      (if (not (eq (send self :cog-balance-check legs-on-grounds) t))
	  (return-from :rate-balance 0.0)
	)
      (dolist (link legs-on-grounds)
	(setq foot-pos
	      (send self :project-on-ground
		    (send (send *aero-lower* (car tmp-legs)
				:end-coords :copy-worldcoords) :pos)))
	(setq edge-length
	      (norm (v- foot-pos
			(send self :project-on-ground
			      (send (send *aero-lower* (cadr tmp-legs)
					  :end-coords :copy-worldcoords) :pos)))))
	(setq get-3foot-center (v+ get-3foot-center foot-pos))
	(setq edge-lengths (push edge-length edge-lengths))
	(setq tmp-legs `(,@(cdr tmp-legs) ,(car tmp-legs)))
	)
      (dolist (x edge-lengths) (setq s (+ s x)) )
      (setq get-3foot-center (scale (/ 1.0 3) get-3foot-center))
      ;; (send (make-coords :pos get-3foot-center) :draw-on :flush t :size 500)
      (setq s (/ s 2))
      (dolist (x edge-lengths) (setq area (* (- s x) area)) )
      (setq area (sqrt (* s area)))
      (setq inner-radius (/ area s))
      (/ inner-radius (norm (v- get-3foot-center com-on-ground)))
      ))



  ;;;;;;;;;;;;;;;;;; for returning values ;;;;;;;;;;;;;;;;;;
  ;; calculate center of mass of the whole robot using
  ;; center of mass of aero upper and aero lower
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
    (send (send (send self :lower limb :end-coords) :copy-worldcoords)
	  :translate (float-vector 0 0 (- wheel-radius)) :world)
    ;; (eval `(send self :lower limb :end-coords))
    )
  (:error-vector ()
    (mapcar 'v- (send *ri* :actual-vector) (send *aero* :angle-vector))
    )
  (:upper-error-sum (&optional (part :all))
    (setq sum 0)
    (setq upper-error-vector (elt (send self :error-vector) 0))
    (case part
          (:rarm
           (setq startIndex 0)
           (setq numJoints 11)
           )
          (:larm
           (setq startIndex 11)
           (setq numJoints 11)
           )
          (otherwise
           (setq startIndex 0)
           (setq numJoints (length upper-error-vector))
           )
          )
    (dotimes (i numjoints)
      (setq sum (+ sum (abs (aref upper-error-vector (+ i startIndex)))))
      )
    sum)



  ;;;;;;;;;;;;;;;;;; for viewing ;;;;;;;;;;;;;;;;;;
  (:draw-on ()
    ;; executing (objects *aero*) is not going to work.
    ;; Use this function instead to visualize aero in irtviewer
    (objects *aero-upper*)
    (objects *aero-lower*)
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    )
  ;; draw center of mass projected onto ground plane
  (:draw-centroid-on-ground ()
    (let ((com-on-ground (make-coords
			  :pos (send self :project-on-ground (send self :get-com))))
          )
      (send com-on-ground :draw-on :flush t)
      com-on-ground))
  (:draw-upper-centroid-on-ground ()
    (let ((com-on-ground (make-coords
			  :pos (send self :project-on-ground (send *aero-upper* :centroid))))
          )
      (send com-on-ground :draw-on :flush t)
      com-on-ground))
  (:draw-lower-centroid-on-ground ()
    (let ((com-on-ground (make-coords
			  :pos (send self :project-on-ground (send *aero-lower* :centroid))))
          )
      (send com-on-ground :draw-on :flush t)
      com-on-ground))
  (:print-lower-error-vector ()
    (setq lower-error-vector (elt error-vector 1))
    (setq lower-joint-names (send *aero-lower* :joint-list :name))
    (print "------- lower ------")
    (dotimes (i (length lower-error-vector))
      (format t "~s ~a~%" (elt lower-joint-names i) (aref lower-error-vector i))
      )
    )
  (:print-upper-error-vector (&optional (part :all))
    (setq upper-joint-names (send *aero-upper* :joint-list :name))
    (setq upper-error-vector (elt (send self :error-vector) 0))
    (case part
          (:rarm
           (setq startIndex 0)
           (setq numJoints 11)
           )
          (:larm
           (setq startIndex 11)
           (setq numJoints 11)
           )
          (otherwise
           (setq startIndex 0)
           (setq numJoints (length upper-error-vector))
           )
          )
    (format t "------- upper ~a ------~%" part)
    (dotimes (i numjoints)
      (format t "~s ~a~%"
	      (elt upper-joint-names (+ i startIndex)) (aref upper-error-vector (+ i startIndex)))
      )
    (send self :print-upper-error-sum part)
    )
  (:print-upper-error-sum (&optional (part :all))
    (format t "~a sum ~a~%" part (send self :upper-error-sum part))
    )
  (:print-joint-error (upperOrLower jointID)
    (setq error-vector (send self :error-vector))
    (if (equal upperOrLower :upper)
        (setq error-vector (elt error-vector 0))
      (setq error-vector (elt error-vector 1))
      )
    (setq joint-names (send self upperOrLower :joint-list :name))
    (format t "~s ~a~%" (elt joint-names jointID) (aref error-vector jointID))
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
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    )
  (:match-ri ()
    (send self :angle-vector (send *ri* :actual-vector))
    (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
    )


 
  );; end of defmethod AeroRobot-robot



(provide :aero "package://aero_ros_bridge/euslisp/aero.l")
