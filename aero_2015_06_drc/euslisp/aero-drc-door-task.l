(load "package://aero_2015_06_drc/euslisp/aero-drc-testbed.l")

(make-drc-testbed-models-with-aero)
(setq *robot* *aero-upper*)

(defun move-waist-with-4legs
  (&optional (waist-velocity (float-vector 0 0 0)))
  (let ((original-end-coords-list) original-angle-vector
        original-coords)
    (setf original-angle-vector (send *aero-lower* :angle-vector))
    (setf original-coords (send *aero-lower* :copy-worldcoords))
    (dolist (link (list :rarm :larm :rleg :lleg))
      (push (send *aero-lower* link :end-coords :copy-worldcoords)
            original-end-coords-list))
    (setf original-end-coords-list (nreverse original-end-coords-list))

    (send *aero-lower* :translate waist-velocity)

    (dolist (link (list :rarm :larm :rleg :lleg))
      (unless
          (send *aero-lower* link :inverse-kinematics
                (car original-end-coords-list) :rotation-axis nil)
        (warn ";; move-waist-with-4legs IK failed >> ~S" link)
        (send *aero-lower* :angle-vector original-angle-vector)
        (send *aero-lower* :newcoords original-coords)
        (send *irtviewer* :draw-objects)
        (return nil)
        )
      (setf original-end-coords-list (cdr original-end-coords-list))
      )
    (send *irtviewer* :draw-objects)
    ))

(defun setup ()
  ; set door
  (send (send *door* :joint :door-hinge-joint) :joint-angle 0)
  (send (send *door* :joint :door-knob-joint) :joint-angle 0)
  ; set poses
  (defvar *rarm-avoid-pose* #f(40.1007 -29.388 4.91744 -71.6494 -22.7883 -2.70865 7.77037 15.0 0.0 0.0 0.0))
  ; set to initial position
  (setq *init-standing-coords*
	(make-coords :pos (float-vector -500 500 0) ; -500 500 -150
		     :rpy (list (deg2rad 160) 0 0))) ; (degrad -20) 0 0
  (send *aero-lower* :fix-leg-to-coords
	(send (send *door* :copy-worldcoords)
	      :transform *init-standing-coords* :local))
  (move-waist-with-4legs #f(0 0 -150))
  (send *aero-upper* :fix-upper-to-coords)
  )

(defun initial-pose ()
  (send *robot* :reset-manip-pose)
  (send *robot* :rarm :angle-vector *rarm-avoid-pose*)
  ;; (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  )

; test-drc-door-task takes a step-by-step fullbody-ik to grasp the knob
; memorizes the posture, and sends angle-vector at once for the real robot
; because aero's body is smaller than hrp2, we might need a different plan
; instead of trying ik, this code tries to reach as near the knob as possible 
(defun reach-for-knob ()
  (setq diff-to-door-knob (v- (send (send
				     (send *door* :joint :door-knob-joint)
				     :child-link)
				    :worldpos)
			      (send *robot* :larm :end-coords :worldpos))
	)
  (send *robot* :reach-arm :larm :stop (abs (elt diff-to-door-knob 0)) :toward (float-vector 1 0 0) :adjust #f(0 0 -1))
  (setq diff-to-door-knob (v- (send (send
				     (send *door* :joint :door-knob-joint)
				     :child-link)
				    :worldpos)
			      (send *robot* :larm :end-coords :worldpos))
	)
  (send *robot* :reach-arm :larm :stop (abs (elt diff-to-door-knob 1)) :toward (float-vector 0 1 0) :adjust #f(0 0 -1))
  (setq diff-to-door-knob (v- (send (send
				     (send *door* :joint :door-knob-joint)
				     :child-link)
				    :worldpos)
			      (send *robot* :larm :end-coords :worldpos))
	)
  (send *robot* :reach-arm :larm :stop (abs (elt diff-to-door-knob 2)) :toward (float-vector 0 0 1) :adjust #f(-1 0 0))
  ;; (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  )

(defun open-door ()
  (send (send *door* :joint :door-knob-joint) :joint-angle -55)
  (send *robot* :larm :move-end-pos #f(0 0 -50))
  (send (send *door* :joint :door-hinge-joint) :joint-angle -15)
  (send *robot* :larm :move-end-pos #f(-150 -130 -30))
  ;; (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  )

(defun test-code ()
  (make-coords :pos (send (send (send *door* :joint :door-knob-joint) :child-link) :worldpos))
  (send *robot* :torso :waist-y :joint-angle -20)
  (send *robot* :larm :wrist-r :joint-angle 0)
  (send *robot* :larm :wrist-p :joint-angle 0)
  )