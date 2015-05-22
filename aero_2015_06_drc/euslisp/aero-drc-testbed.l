(load "package://drc_task_common/euslisp/drc-testbed-models.l")
(load "package://aero_ros_bridge/euslisp/aero-interface.l")

(defun make-drc-testbed-models-with-aero ()
  (aero-init)
  (send *aero* :locate (send (make-coords) :pos) :world)
  (send *aero* :translate (float-vector -1000 500 0))
  (send *aero* :reset-manip-pose)
  ;; Generate models                 
  (make-drc-door)
  (make-drc-hose-plug)
  (make-drc-hose-wall)
  (make-drc-stair)
  (make-drc-drill-wall)
  (make-drc-terrain)
  (make-drc-valve)
  (let ((sb (make-drc-surprise-task-box)))
    (send-all sb :translate (float-vector 12000 0 0) :world))
  ;; Align models               
  (send *stair* :translate (float-vector 8000 0 0) :world)
  (send *door* :translate (float-vector 0 0 0) :world)
  (send *hose-wall* :rotate (deg2rad -45) :z)
  (send *hose-wall* :translate (float-vector 4000 -3000 0) :world)
  (send *drill-wall* :rotate -pi/2 :z)
  (send *drill-wall* :translate (float-vector 2000 -3500 0) :world)
  (send *terrain* :translate (float-vector 5000 -1000 0) :world)
  (send *drc-valve* :rotate -pi/2 :y)
  (send *drc-valve* :rotate (deg2rad -135) :x)
  (send *drc-valve* :translate (float-vector 2000 -3500 1130) :world)
  (send *drc-valve* :translate (float-vector 0 (/ 1210 -2.0) (+ -110 -400)))
  ;; Display
  (objects (list *hose-plug* *hose-wall* *stair* *door* *drill-wall* *terrain* *drc-valve*
                 *surprise-task-shower* *surprise-task-button* *surprise-task-lever* *surprise-task-rope*))
  (send *aero* :draw-on)
  )

;; utils

(defvar *exit-flag* nil)

(defun go? (cmd &optional (real *real*))
  (let (ui) ; ui is user input
    (if (eq *exit-flag* t)
	(return-from go? "you've got failed"))
    (eval cmd)
    (print cmd)
    (send *irtviewer* :draw-objects)
    (cond ((eq real t)
	   (print "send real?")
	   (setq ui (read-line))
	   (if (or (equal ui "exit") (equal ui "q") (equal ui "no"))
	       (progn (setq *exit-flag* t)
		      (return-from go? (warn "detected cancelation~%")))
	     (send *ri* :angle-vector (send *aero* :stroke-vector))))
	  (t
	   (setq ui (read-line))
	   (if (or (equal ui "exit") (equal ui "q") (equal ui "no"))
	       (progn (setq *exit-flag* t)
		      (return-from go? (warn "detected cancelation~%")))))
	  )
    ))

(defun init-testbed (object
		     &key (pos #f(0 0 0)) (rpy (list 0 0 0)))
  (let (init-standing-coords)
    (setq init-standing-coords
	  (make-coords :pos (float-vector (aref pos 0) (aref pos 1) 0)
		       :rpy rpy))
    (send *aero* :locate
	  (send (send (send object :copy-worldcoords)
		      :transform init-standing-coords :local)
		:pos) :world)
    (if (not (equal rpy (list 0 0 0)))
	(send *aero* :rotate
	      (car (rotation-angle
		    (send (send (send object :copy-worldcoords)
				:transform init-standing-coords :local)
			  :rot))) #f(0 0 1)))
    (if (not (= (aref pos 2) 0))
	(send *aero* :move-waist (float-vector 0 0 (aref pos 2))))
    ))

(defun reset-aero ()
  (send *aero* :locate (send (make-coords) :pos) :world)
  (send *aero* :translate (float-vector -1000 500 0))
  (go? `(send *aero* :reset-manip-pose))
  )
