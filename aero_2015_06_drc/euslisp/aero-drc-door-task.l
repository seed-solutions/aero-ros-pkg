(load "package://aero_2015_06_drc/euslisp/aero-drc-testbed.l")

(make-drc-testbed-models-with-aero)
(objects (list *door*))
(send *aero* :draw-on)

(defun setup ()
  ; set door
  (send (send *door* :joint :door-hinge-joint) :joint-angle 0)
  (send (send *door* :joint :door-knob-joint) :joint-angle 0)
  ; set poses
  (defvar *rarm-avoid-pose* #f(40.1007 -29.388 4.91744 -71.6494 -22.7883 -2.70865 7.77037 15.0 0.0 0.0 0.0))
  ; set to initial position
  (setq *init-standing-coords*
	(make-coords :pos (float-vector -500 500 0)
		     :rpy (list (deg2rad -20) 0 0)))
  (send *aero* :locate
  	(send (send (send *door* :copy-worldcoords)
		    :transform *init-standing-coords* :local)
	      :pos) :world)
  (send *aero* :rotate
	(car (rotation-angle
	      (send (send (send *door* :copy-worldcoords)
			  :transform *init-standing-coords* :local)
		    :rot))) #f(0 0 1))
  (send *aero* :move-waist #f(0 0 -150))
  )

(defun initial-pose ()
  (send *aero* :upper :reset-manip-pose)
  (send *aero* :upper :rarm :angle-vector *rarm-avoid-pose*)
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :angle-vector (send *aero* :stroke-vector))
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
			      (send *aero* :upper :larm :end-coords :worldpos))
	)
  (send *aero* :reach-arm :larm :stop (abs (elt diff-to-door-knob 0)) :toward (float-vector 1 0 0) :adjust #f(0 0 -1))
  (send *irtviewer* :draw-objects)
  (setq diff-to-door-knob (v- (send (send
				     (send *door* :joint :door-knob-joint)
				     :child-link)
				    :worldpos)
			      (send *aero* :upper :larm :end-coords :worldpos))
	)
  (send *aero* :reach-arm :larm :stop (abs (elt diff-to-door-knob 1)) :toward (float-vector 0 1 0) :adjust #f(0 0 -1))
  (send *irtviewer* :draw-objects)
  (setq diff-to-door-knob (v- (send (send
				     (send *door* :joint :door-knob-joint)
				     :child-link)
				    :worldpos)
			      (send *aero* :upper :larm :end-coords :worldpos))
	)
  (send *aero* :reach-arm :larm :stop (abs (elt diff-to-door-knob 2)) :toward (float-vector 0 0 1) :adjust #f(-1 0 0))
  (send *irtviewer* :draw-objects)
  ;; (send *ri* :angle-vector (send *aero* :stroke-vector))
  )

(defun open-door ()
  (send (send *door* :joint :door-knob-joint) :joint-angle -55)
  (send *aero* :upper :larm :move-end-pos #f(0 0 -50))
  (send (send *door* :joint :door-hinge-joint) :joint-angle -15)
  (send *aero* :upper :larm :move-end-pos #f(-150 -130 -30))
  ;; (send *ri* :angle-vector (send *aero* :stroke-vector))
  )
