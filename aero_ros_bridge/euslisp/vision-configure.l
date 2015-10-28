#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(load "package://aeroeus/aero-interface.l")
;; (load "package://aero_ros_bridge/euslisp/tf.l")

(aero-init)

(setq ((send *aero* :head :neck-r) . min-angle) 0)
(setq ((send *aero* :head :neck-r) . max-angle) 0)

(setq l-hand-center #f(-50 20 0))
(setq r-hand-center #f(-50 -20 0))

(setq *tl* (instance ros::transform-listener :init))

(defun ~tf (&optional (arm nil))
  (let (m
        )
    (ros::spin-once)
    (setq m (send *tl* :lookup-transform "leg_base_link" "tomato" (ros::time)))
    (if (eq m nil) (return-from ~tf nil))
    (setq m (make-coords :pos (v+ (send m :pos)
				  (send (send (car (send *aero* :links))
					      :copy-worldcoords) :pos))))
    (send m :draw-on :flush t)
    m))


(defun setup-limits (arm &key (torso nil))
  (let (origin
	limit-search
	(result nil)
	)
    (send *aero* :reset-manip-pose)
    (setq origin (send *aero* arm :end-coords :copy-worldcoords))
    ;; z-
    (setq limit-search (make-coords
			:pos (v+ (send origin :pos) #f(0 0 -10000))))
    (if (eq torso t)
	(send *aero* arm :inverse-kinematics limit-search
	      :revert-if-fail nil
	      :link-list (send *aero* :link-list
			       (send *aero* arm :end-coords :parent)))
      (send *aero* arm :inverse-kinematics limit-search
	    :revert-if-fail nil)
      )
    (push (elt (send 
		(send *aero* arm :end-coords :copy-worldcoords)
		:pos) 2) result) 
    ;; z+
    (setq limit-search (make-coords
			:pos (v+ (send origin :pos) #f(0 0 10000))))
    (if (eq torso t)
	(send *aero* arm :inverse-kinematics limit-search
	      :revert-if-fail nil
	      :link-list (send *aero* :link-list
			       (send *aero* arm :end-coords :parent)))
      (send *aero* arm :inverse-kinematics limit-search
	    :revert-if-fail nil)
      )
    (push (elt (send 
		(send *aero* arm :end-coords :copy-worldcoords)
		:pos) 2) result)
    ;; y-
    (setq limit-search (make-coords
			:pos (v+ (send origin :pos) #f(0 -10000 0))))
    (if (eq torso t)
	(send *aero* arm :inverse-kinematics limit-search
	      :revert-if-fail nil
	      :link-list (send *aero* :link-list
			       (send *aero* arm :end-coords :parent)))
      (send *aero* arm :inverse-kinematics limit-search
	    :revert-if-fail nil)
      )
    (push (elt (send 
		(send *aero* arm :end-coords :copy-worldcoords)
		:pos) 1) result) 
    ;; y+
    (setq limit-search (make-coords
			:pos (v+ (send origin :pos) #f(0 10000 0))))
    (if (eq torso t)
	(send *aero* arm :inverse-kinematics limit-search
	      :revert-if-fail nil
	      :link-list (send *aero* :link-list
			       (send *aero* arm :end-coords :parent)))
      (send *aero* arm :inverse-kinematics limit-search
	    :revert-if-fail nil)
      )
    (push (elt (send 
		(send *aero* arm :end-coords :copy-worldcoords)
		:pos) 1) result)
    ;; x-
    (setq limit-search (make-coords
			:pos (v+ (send origin :pos) #f(-10000 0 0))))
    (if (eq torso t)
	(send *aero* arm :inverse-kinematics limit-search
	      :revert-if-fail nil
	      :link-list (send *aero* :link-list
			       (send *aero* arm :end-coords :parent)))
      (send *aero* arm :inverse-kinematics limit-search
	    :revert-if-fail nil)
      )
    (push (elt (send 
		(send *aero* arm :end-coords :copy-worldcoords)
		:pos) 0) result) 
    ;; x+
    (setq limit-search (make-coords
			:pos (v+ (send origin :pos) #f(10000 0 0))))
    (if (eq torso t)
	(send *aero* arm :inverse-kinematics limit-search
	      :revert-if-fail nil
	      :link-list (send *aero* :link-list
			       (send *aero* arm :end-coords :parent)))
      (send *aero* arm :inverse-kinematics limit-search
	    :revert-if-fail nil)
      )
    (push (elt (send 
		(send *aero* arm :end-coords :copy-worldcoords)
		:pos) 0) result)
    result))


(defun check-all-ik (arm &key (torso nil) (with-head nil) (bits 100.0)
			 (p-bits 10.0) (y-bits 20.0))
  (let (arm-limits
	x-steps y-steps z-steps
	target target-origin
	head-p head-y
	(n-1-found (send (make-coords) :pos))
	(n-found nil)
	(i-saw-ghost nil)
	(arm-list-actual nil)
	(arm-list-model nil)
	)
    (setq head-p (send *aero* :head :neck-p :joint-angle))
    (setq head-y (send *aero* :head :neck-y :joint-angle))
    (cond ((eq arm :larm)
	   (setq arm-limits (setup-limits :larm :torso torso))
	   (send *aero* :rarm :shoulder-r :joint-angle -30)
	   (send *aero* :rarm :shoulder-y :joint-angle -90)
	   )
	  (t
	   (setq arm-limits (setup-limits :rarm :torso torso))
	   (send *aero* :larm :shoulder-r :joint-angle 30)
	   (send *aero* :larm :shoulder-y :joint-angle 90)
	   )
	  )
    (send *aero* :head :neck-p :joint-angle head-p)
    (send *aero* :head :neck-y :joint-angle head-y)
    (setq x-steps (/ (- (elt arm-limits 0) (elt arm-limits 1)) bits))
    (setq y-steps (/ (- (elt arm-limits 2) (elt arm-limits 3)) bits))
    (setq z-steps (/ (- (elt arm-limits 4) (elt arm-limits 5)) bits))
    (setq target-origin (make-coords
			 :pos (float-vector (elt arm-limits 1)
					    (elt arm-limits 3)
					    (elt arm-limits 5))))
    (setq target (send target-origin :copy-worldcoords))
    (if (eq torso t)
	(send *aero* arm :inverse-kinematics target
	      :revert-if-fail nil
	      :link-list (send *aero* :link-list
			       (send *aero* arm :end-coords :parent)))
      (send *aero* arm :inverse-kinematics target
	    :revert-if-fail nil)
      )
    (cond ((eq *real* t)
	   (send *ri* :angle-vector
	   	 (send *aero* :stroke-vector) 2000)
	   (unix::usleep (* 4000 1000))
	   )
	  )
    (send *irtviewer* :draw-objects)
    (dotimes (j y-steps)
      (dotimes (k z-steps)
	(dotimes (i x-steps)
	  (setq target (make-coords
			:pos 
			(v+ (send (send target-origin :copy-worldcoords) :pos)
			    (float-vector (* i bits) (* j bits) (* k bits))))) 
	  (if (eq torso t)
	      (send *aero* arm :inverse-kinematics target
		    :revert-if-fail nil
		    :link-list (send *aero* :link-list
				     (send *aero*
					   arm :end-coords :parent)))
	    (send *aero* arm :inverse-kinematics target
		  :revert-if-fail nil)
	    )
	  (send *aero* :head :look-at
		(send (send *aero* arm :end-coords :copy-worldcoords)
		      :pos))
	  (send *irtviewer* :draw-objects)
	  (cond ((eq *real* t)
		 (print "moving arm to pose")
		 (send *ri* :angle-vector
		       (send *aero* :stroke-vector) 2000)
		 (unix::usleep (* 5000 1000))
		 (setq n-found (~tf))
		 (block find-hand
			(cond ((not (eq n-found nil))
			       (setq n-found (send n-found :pos))
			       (if (and
				    (eps= (elt n-found 0) (elt n-1-found 0) 1.0)
				    (eps= (elt n-found 1) (elt n-1-found 1) 1.0)
				    (eps= (elt n-found 2) (elt n-1-found 2) 1.0))
				   (progn (print "I see ghost")
					  (cond ((eq i-saw-ghost nil)
						 (pop arm-list-model)
						 (pop arm-list-actual)
						 (setq i-saw-ghost t)
						 ))
					  (return-from find-hand))
				 )
			       (let ((tmp-hand (send *aero* arm
						     :end-coords
						     :copy-worldcoords))
				     )
				 (push (concatenate float-vector
					(v+ (send tmp-hand :pos)
					    (transform (send tmp-hand :rot)
						       (if (eq arm :larm)
							   l-hand-center
							 r-hand-center))
					    )
					(float-vector
					 (send *aero*
					       :head :neck-p :joint-angle)
					 (send *aero*
					       :head :neck-y :joint-angle)
					 )
					) arm-list-model))
			       (push n-found arm-list-actual)
			       (print (format nil "found ~A" n-found))
			       (setq n-1-found n-found)
			       (setq i-saw-ghost nil)
			       t)
			       )
			)
		 )
		)
	  (cond ((eq with-head t)
		 (setq head-p (send *aero* :head :neck-p :joint-angle))
		 (setq head-y (send *aero* :head :neck-y :joint-angle))
		 (dotimes (l 3)
		   (send *aero* :head :neck-p :joint-angle
			 (+ head-p (* (- l 1) p-bits)))
		   (dotimes (m 3)
		     (send *aero* :head :neck-y :joint-angle
			   (+ head-y (* (- m 1) y-bits)))
		     (send *irtviewer* :draw-objects)
		     (cond ((eq *real* t)
			    (send *ri* :angle-vector
			    	  (send *aero* :stroke-vector) 2000)
			    (unix::usleep (* 4000 1000))
			    )
			   )
		     ))
		 )
		)
	  )))
    (list arm-list-model arm-list-actual)))

(defun save-calib (name data)
  (with-open-file (str (concatenate string name "_in.txt") :direction :output
		       :if-exists :supersede)
		  (dolist (x (car data))
		    (format str "~A ~A ~A ~A ~A~%"
			    (elt x 0) (elt x 1) (elt x 2) (elt x 3) (elt x 4))
		    )
		  )
  (with-open-file (str (concatenate string name "_x_out.txt") :direction :output
		       :if-exists :supersede)
		  (dolist (x (cadr data))
		    (format str "~A~%" (elt x 0))
		    )
		  )
  (with-open-file (str (concatenate string name "_y_out.txt") :direction :output
		       :if-exists :supersede)
		  (dolist (x (cadr data))
		    (format str "~A~%" (elt x 1))
		    )
		  )
  (with-open-file (str (concatenate string name "_z_out.txt") :direction :output
		       :if-exists :supersede)
		  (dolist (x (cadr data))
		    (format str "~A~%" (elt x 2))
		    )
		  )
  (with-open-file (str (concatenate string name "_test.txt") :direction :output
		       :if-exists :supersede)
		  (dotimes (x (length (car data)))
		    (let (tmp
			  )
		      (setq tmp (elt (car data) x))
		      (format str "~A 1:~A 2:~A 3:~A 4:~A 5:~A~%"
			      (elt (elt (cadr data) x) 0)
			      (elt tmp 0) (elt tmp 1) (elt tmp 2)
			      (elt tmp 3) (elt tmp 4))
		      )
		    )
		  )
  )
