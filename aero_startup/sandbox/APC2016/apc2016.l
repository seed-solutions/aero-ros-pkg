#!/usr/bin/env roseus

(ros::load-ros-manifest "aero_startup")
(load "package://aeroeus/aero-interface.l")
(load "kiva-pod-model.l")
(load "tote-model.l")

(setq *real* nil)

(setq *workcell* (make-cube 2000 2000 10))
(send *workcell* :translate (float-vector 0 0 -5))
(send *workcell* :set-color :yellow)

(unless (boundp '*kiva-pod*)
  (setq *kiva-pod* (make-kiva-pod))
  (send *kiva-pod* :translate (float-vector 1000 0 0))
  (send *kiva-pod* :rotate pi :z))

(unless (boundp '*tote*)
  (setq *tote* (make-tote))
  (send *tote* :translate (float-vector 0 400 700))
  (setq *tote-target*
	(make-cascoords :coords (send *tote* :copy-worldcoords)))
  (send *tote-target* :translate (float-vector 80 0 80))
  (send *tote-target* :rotate -pi/2 :y)
  (send *tote* :assoc *tote-target*))

(unless (boundp '*aero*)
  (if *real*
      (progn
	(aero-init)
	(load-controllers))
    (progn
      (setq *aero* (instance AeroUpperRobot-robot :init))))
  (send *aero* :translate (float-vector 0 0 850))

  (setq *aero-lower* (make-cube 100 100 650))
  (send *aero-lower* :set-color :gray)
  (send *aero-lower* :translate (float-vector 0 0 525))

  (setq *aero-base* (make-cube 720 720 500))
  (send *aero-base* :set-color :gray)
  (send *aero-base* :translate (float-vector 0 0 250))

  (send *aero* :assoc *tote*)
  (send *aero* :assoc *aero-lower*)
  )

(objects (list *workcell* *aero* *aero-lower* *aero-base* *kiva-pod* *tote*)))


(defun ik-with-torso (coords)
  (when
      (send *aero* :rarm :inverse-kinematics coords
	    :rotation-axis :z :torso t
	    :link-list
	    (send *aero* :link-list
		  (send *aero* :rarm :end-coords :parent)
		  (car (send *aero* :torso :links))))
    (send *aero* :larm :elbow-p :joint-angle -120)
    (send *aero* :head :look-at (send *aero* :rarm :end-coords :worldpos))
    ))

;; example
;; (ik-with-torso (send (send *kiva-pod* :spot-h :copy-worldcoords) :translate (float-vector 0 0 100)))


(defun reach (target)
  (let ((left-offs 230) (right-offs -100)
	(abc-height 1200) (def-height 900) (ghi-height 800) (jkl-height 650))
    (cond
     ((eq target :a)
      (send *aero* :locate (float-vector 0 left-offs abc-height) :world)
      (send *aero* :reset-manip-pose)
      (ik-with-torso
       (send
	(send *kiva-pod* :spot-a :copy-worldcoords)
	:translate (float-vector 0 0 100)))
      )
     ((eq target :b)
      (send *aero* :locate (float-vector 0 0 abc-height) :world)
      (send *aero* :reset-manip-pose)
      (ik-with-torso
       (send
	(send *kiva-pod* :spot-b :copy-worldcoords)
	:translate (float-vector 0 0 100)))
      )
     ((eq target :c)
      (send *aero* :locate (float-vector 0 right-offs abc-height) :world)
      (send *aero* :reset-manip-pose)
      (ik-with-torso
       (send
	(send *kiva-pod* :spot-c :copy-worldcoords)
	:translate (float-vector 0 0 100)))
      )
     ((eq target :d)
      (send *aero* :locate (float-vector 0 left-offs def-height) :world)
      (send *aero* :reset-manip-pose)
      (ik-with-torso
       (send
	(send *kiva-pod* :spot-d :copy-worldcoords)
	:translate (float-vector 0 0 100)))
      )
     ((eq target :e)
      (send *aero* :locate (float-vector 0 0 def-height) :world)
      (send *aero* :reset-manip-pose)
      (ik-with-torso
       (send
	(send *kiva-pod* :spot-e :copy-worldcoords)
	:translate (float-vector 0 0 100)))
      )
     ((eq target :f)
      (send *aero* :locate (float-vector 0 right-offs def-height) :world)
      (send *aero* :reset-manip-pose)
      (ik-with-torso
       (send
	(send *kiva-pod* :spot-f :copy-worldcoords)
	:translate (float-vector 0 0 100)))
      )
     ((eq target :g)
      (send *aero* :locate (float-vector 0 left-offs ghi-height) :world)
      (send *aero* :reset-manip-pose)
      (ik-with-torso
       (send
	(send *kiva-pod* :spot-g :copy-worldcoords)
	:translate (float-vector 0 0 100)))
      )
     ((eq target :h)
      (send *aero* :locate (float-vector 0 0 ghi-height) :world)
      (send *aero* :reset-manip-pose)
      (ik-with-torso
       (send
	(send *kiva-pod* :spot-h :copy-worldcoords)
	:translate (float-vector 0 0 100)))
      )
     ((eq target :i)
      (send *aero* :locate (float-vector 0 right-offs ghi-height) :world)
      (send *aero* :reset-manip-pose)
      (ik-with-torso
       (send
	(send *kiva-pod* :spot-i :copy-worldcoords)
	:translate (float-vector 0 0 100)))
      )
     ((eq target :j)
      (send *aero* :locate (float-vector 0 left-offs jkl-height) :world)
      (send *aero* :reset-manip-pose)
      (ik-with-torso
       (send
	(send *kiva-pod* :spot-j :copy-worldcoords)
	:translate (float-vector 0 0 100)))
      )
     ((eq target :k)
      (send *aero* :locate (float-vector 0 0 jkl-height) :world)
      (send *aero* :reset-manip-pose)
      (ik-with-torso
       (send
	(send *kiva-pod* :spot-k :copy-worldcoords)
	:translate (float-vector 0 0 100)))
      )
     ((eq target :l)
      (send *aero* :locate (float-vector 0 right-offs jkl-height) :world)
      (send *aero* :reset-manip-pose)
      (ik-with-torso
       (send
	(send *kiva-pod* :spot-l :copy-worldcoords)
	:translate (float-vector 0 0 100)))
      )
     ((eq target :tote)
      ;; (send *aero* :locate (float-vector 0 0 850) :world)
      (send *aero* :reset-manip-pose)
      (send *aero* :torso :waist-y :joint-angle 90)
      (ik-with-torso *tote-target*)
      )
     )))

(defun reset-aero-position ()
  (send *aero* :locate (float-vector 0 0 850) :world))

(defun reach-and-tote (target)
  (reach target)
  (send *irtviewer* :draw-objects)
  (unix::usleep (* 3000 1000))
  (reach :tote)
  (send *irtviewer* :draw-objects)
  (unix::usleep (* 3000 1000)))
(defun do-sim-demo ()
  (reset-aero-position)
  (send *aero* :reset-manip-pose)
  (send *irtviewer* :draw-objects)
  (unix::usleep (* 3000 1000))

  (reach-and-tote :a)
  (reach-and-tote :b)
  (reach-and-tote :c)
  (reach-and-tote :d)
  (reach-and-tote :e)
  (reach-and-tote :f)
  (reach-and-tote :g)
  (reach-and-tote :h)
  (reach-and-tote :i)
  (reach-and-tote :j)
  (reach-and-tote :k)
  (reach-and-tote :l)

  (reset-aero-position)
  (send *aero* :reset-manip-pose)
  (send *irtviewer* :draw-objects)
  (unix::usleep (* 3000 1000))
  )
