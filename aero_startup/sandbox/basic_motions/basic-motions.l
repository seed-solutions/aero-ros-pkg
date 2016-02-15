#!/usr/bin/env roseus

(ros::load-ros-manifest "aero_startup")
(load "package://aeroeus/aero-interface.l")
(load-controllers)
(load "package://aeroeus/pre-release/inverse-kinematics-extension.l")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun start-pose (&optional (time 3000))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 0.0 0.0))
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
    (cond ((eq *real* t)
	   (send *ri* :angle-vector (send *aero* :angle-vector) time)
	   (unix::usleep (* time 1000))
	   ))
    (send *aero* :angle-vector b-pose)
    (cond ((eq *real* t)
	   (send *ri* :angle-vector (send *aero* :angle-vector) (* time 0.5))
	   (unix::usleep (* time 500))
	   ))
    (send *aero* :angle-vector a-pose)
    (cond ((eq *real* t)
	   (send *ri* :angle-vector (send *aero* :angle-vector) (* time 0.5))
	   (unix::usleep (* time 500))
	   ))
    (send *aero* :angle-vector b-pose)
    (cond ((eq *real* t)
	   (send *ri* :angle-vector (send *aero* :angle-vector) (* time 0.5))
	   (unix::usleep (* time 500))
	   ))
    (send *aero* :angle-vector a-pose)
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
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) 500)
	 (unix::usleep (* 500 1000))
	 ))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 20.0 100.0 -80.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 40.0 0.0))
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 20.0 0.0 -100.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 40.0 0.0))
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
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 120.0 -30.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 0.0 0.0))
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  (send *aero* :angle-vector
	#f(0.0 -2.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 -10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
	   0.0 0.0 0.0 0.0 0.0 0.0))
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) 500)
	 (unix::usleep (* 500 1000))
	 ))
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun nod-no (&optional (time 500))
  (send *aero* :head :neck-y :joint-angle -30)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  (send *aero* :head :neck-y :joint-angle 30)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) (* time 2))
	 (unix::usleep (* time 2000))
	 ))
  (send *aero* :head :neck-y :joint-angle 0)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun nod-yes (&optional (time 500))
  (send *aero* :head :neck-p :joint-angle 40)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  (send *aero* :head :neck-p :joint-angle 0)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  (send *aero* :head :neck-p :joint-angle 40)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  (send *aero* :head :neck-p :joint-angle 0)
  (cond ((eq *real* t)
	 (send *ri* :angle-vector (send *aero* :angle-vector) time)
	 (unix::usleep (* time 1000))
	 ))
  )


(defun ojigi ()
  (start-pose)
  (sorry 2000)
  (start-pose))


(defun appear-pose-0 ()
  (send *aero* :larm :angle-vector
	#f(-50.0 15.0 20.0 -120.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0))
  (send *aero* :rarm :angle-vector
	#f(-50.0 -15.0 -20.0 -120.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0)))

(defun appear-pose-1 ()
  (send *aero* :larm :angle-vector
	#f(-50.0 30.0 40.0 -110.0 10.0 0.0 10.0 0.0 0.0 0.0 0.0)
	)
  (send *aero* :rarm :angle-vector
	#f(-50.0 -30.0 -40.0 -110.0 -10.0 0.0 -10.0 0.0 0.0 0.0 0.0)
	))

(defun look (&optional (side :left))
  (cond
   ((eq side :left)
    (send *aero* :head :neck-p :joint-angle 10)
    (send *aero* :head :neck-y :joint-angle 15)
    (send *aero* :torso :waist-y :joint-angle 15)
    )
   ((eq side :right)
    (send *aero* :head :neck-p :joint-angle 10)
    (send *aero* :head :neck-y :joint-angle -15)
    (send *aero* :torso :waist-y :joint-angle -15)
    ))
  )

(defun appear (&key (swing-time 500))
  (send *aero* :reset-manip-pose)
  (send *ri* :angle-vector (send *aero* :angle-vector) 3000)
  (unix::usleep (* 3000 1000))

  ;; turn-left
  (appear-pose-0)
  (look :left)
  (send *ri* :angle-vector (send *aero* :angle-vector) 1500)
  (unix::usleep (* 1500 1000))

  (appear-pose-1)
  (send *ri* :angle-vector (send *aero* :angle-vector) swing-time)
  (unix::usleep (* swing-time 1000))
  (appear-pose-0)
  (send *ri* :angle-vector (send *aero* :angle-vector) swing-time)
  (unix::usleep (* swing-time 1000))
  (appear-pose-1)
  (send *ri* :angle-vector (send *aero* :angle-vector) swing-time)
  (unix::usleep (* swing-time 1000))
  

  ;; turn right
  (appear-pose-0)
  (look :right)
  (send *ri* :angle-vector (send *aero* :angle-vector) 1200)
  (unix::usleep (* 1200 1000))
  
  (appear-pose-1)
  (send *ri* :angle-vector (send *aero* :angle-vector) swing-time)
  (unix::usleep (* swing-time 1000))
  (appear-pose-0)
  (send *ri* :angle-vector (send *aero* :angle-vector) swing-time)
  (unix::usleep (* swing-time 1000))
  (appear-pose-1)
  (send *ri* :angle-vector (send *aero* :angle-vector) swing-time)
  (unix::usleep (* swing-time 1000))


  ;; end
  (send *aero* :reset-manip-pose)
  (appear-pose-0)
  (send *ri* :angle-vector (send *aero* :angle-vector) 1300)
  (unix::usleep (* 1300 1000))

  (send *aero* :reset-manip-pose)
  (send *ri* :angle-vector (send *aero* :angle-vector) 1500)
  (unix::usleep (* 1500 1000))

  ;; (start-pose)
  )

(aero-init)
(objects *aero*) ;; this has to be called or error occurs
(setq *real* t)


(defun usage ()
  (format t ";; (start-pose)~%")
  (format t ";; (ojigi)~%")
  (format t ";; (appear)~%"))
(usage)


(defun pose-until-key
  (&optional (sleep-time (* 60 1000 1000)))
  (let (sleep-cnt sleep-tick)
    ;; (start-pose)
    (setq sleep-cnt 0
	  sleep-tick (* 1000 1000))
    (do-until-key
     (unix:usleep sleep-tick)
     (setq sleep-cnt (+ sleep-cnt sleep-tick))
     (format t ";; sleep ~a / ~a~%" sleep-cnt sleep-time)

     (when (> sleep-cnt sleep-time)
       (ojigi)
       (appear)
       (setq sleep-cnt 0)
       )
     )
    )
  )
