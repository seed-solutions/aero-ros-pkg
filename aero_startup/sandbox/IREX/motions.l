;; @file motions.l
;; @brief robot motions for IREX demo
;; fucntions:
;;  (start-pose)
;;  (run-pose)
;;  (manip-pose)
;;  (end-pose)
;;  (sorry)
;;  (nod-yes)
;;  (nod-no)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun start-pose (&optional (time 3000))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 0.0 0.0))
  (send *irtviewer* :draw-objects)
  (cond ((eq *real* t)
	 (send *ri* :move-waist #f(0 0 0) :world)
	 (unix::usleep (* 2200 1000))
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun sorry (&optional (time 1000))
  (let ((a-pose #f(0.0 -2.0 0.0 -30.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
		   0.0 0.0 0.0 -30.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
		   0.0 30.0 0.0 0.0 40.0 0.0))
	(b-pose #f(-0.035975 -2.05717 0.596428 -26.5836 -0.26772 6.61294 -0.036075 0.0 0.0 0.0 0.0
		   -0.038505 0.005706 -0.022469 -26.8227 0.026125 6.86112 0.006531 0.0 0.0 0.0 0.0
		   0.0 20.0 0.0 0.0 40.0 0.0))
	)
    (send *aero* :angle-vector a-pose)
    (send *irtviewer* :draw-objects)
    (cond ((eq *real* t)
	   (send *ri* :angle-vector (send *aero* :angle-vector) time)
	   (unix::usleep (* time 1000))
	   ))
    (send *aero* :angle-vector b-pose)
    (send *irtviewer* :draw-objects)
    (cond ((eq *real* t)
	   (send *ri* :angle-vector (send *aero* :angle-vector) (* time 0.5))
	   (unix::usleep (* time 500))
	   ))
    (send *aero* :angle-vector a-pose)
    (send *irtviewer* :draw-objects)
    (cond ((eq *real* t)
	   (send *ri* :angle-vector (send *aero* :angle-vector) (* time 0.5))
	   (unix::usleep (* time 500))
	   ))
    (send *aero* :angle-vector b-pose)
    (send *irtviewer* :draw-objects)
    (cond ((eq *real* t)
	   (send *ri* :angle-vector (send *aero* :angle-vector) (* time 0.5))
	   (unix::usleep (* time 500))
	   ))
    (send *aero* :angle-vector a-pose)
    (send *irtviewer* :draw-objects)
    (cond ((eq *real* t)
	   (send *ri* :angle-vector (send *aero* :angle-vector) (* time 0.5))
	   (unix::usleep (* time 500))
	   ))
    ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun run-pose (&optional (time 2000))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 40.0 0.0))
  (send *irtviewer* :draw-objects)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun manip-pose (&optional (time 1000))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 120.0 -30.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 40.0 0.0))
  (send *irtviewer* :draw-objects)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) 500)
	 (unix::usleep (* 500 1000))
	 ))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 20.0 100.0 -80.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 40.0 0.0))
  (send *irtviewer* :draw-objects)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 20.0 0.0 -100.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 40.0 0.0))
  (send *irtviewer* :draw-objects)
  (cond ((eq *real* t)
         (send *ri* :angle-vector (send *aero* :angle-vector) time)
         (unix::usleep (* time 1000))
         ))
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun back-pose (&optional (time 2000))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 20.0 0.0 -100.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 40.0 0.0))
  (send *irtviewer* :draw-objects)
  (cond ((eq *real* t)
	 (send *ri* :move-waist #f(0 0 0) :world)
	 (unix::usleep (* 2200 1000))
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun end-pose (&optional (time 1000))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 20.0 0.0 -100.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 0.0 0.0))
  (send *irtviewer* :draw-objects)
  (cond ((eq *real* t)
	 (send *ri* :move-waist #f(0 0 0) :world)
	 (unix::usleep (* 2200 1000))
	 (send *ri* :angle-vector (send *aero* :angle-vector) 5000)
	 (unix::usleep (* 5000 1000))
	 ))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 20.0 100.0 -80.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 0.0 0.0))
  (send *irtviewer* :draw-objects)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 120.0 -30.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 0.0 0.0))
  (send *irtviewer* :draw-objects)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 0.0 0.0))
  (send *irtviewer* :draw-objects)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) 500)
	 (unix::usleep (* 500 1000))
	 ))
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun nod-no (&optional (time 500))
  (send *aero* :head :neck-y :joint-angle -30)
  (send *irtviewer* :draw-objects)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  (send *aero* :head :neck-y :joint-angle 30)
  (send *irtviewer* :draw-objects)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) (* time 2))
	 (unix::usleep (* time 2000))
	 ))
  (send *aero* :head :neck-y :joint-angle 0)
  (send *irtviewer* :draw-objects)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun nod-yes (&optional (time 500))
  (send *aero* :head :neck-p :joint-angle 40)
  (send *irtviewer* :draw-objects)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  (send *aero* :head :neck-p :joint-angle 0)
  (send *irtviewer* :draw-objects)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  (send *aero* :head :neck-p :joint-angle 40)
  (send *irtviewer* :draw-objects)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  (send *aero* :head :neck-p :joint-angle 0)
  (send *irtviewer* :draw-objects)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  )
