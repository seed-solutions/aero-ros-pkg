(load "package://aero_2015_06_drc/euslisp/aero-drc-testbed.l")

(make-drc-testbed-models-with-aero)

(defun setup-beta ()
  ; set door
  (send (send *door* :joint :door-hinge-joint) :joint-angle 0)
  (send (send *door* :joint :door-knob-joint) :joint-angle 0)
  ; set to initial position
  (init-testbed *door* :pos #f(-500 500 -150) :rpy (list (deg2rad -20) 0 0))
  (objects (list *door*))
  (send *aero* :draw-on)
  )

(defun setup-v1.00 ()
  ; set door
  (send (send *door* :joint :door-hinge-joint) :joint-angle 0)
  (send (send *door* :joint :door-knob-joint) :joint-angle 0)
  ; set to initial position
  (init-testbed *door* :pos #f(-500 500 0) :rpy (list 0 0 0))
  (objects (list *door*))
  (send *aero* :draw-on)
  )

;; functions

(defun hand-yaw-world (arm angle)
  (send *aero* :upper arm :wrist-y :joint-angle 0)
  (send *aero* :upper arm :wrist-y :joint-angle
	(+ (* (car (rotation-angle
		    (send (send (send *aero* :upper arm :end-coords)
				:copy-worldcoords) :rot)))
	   (if (eq arm :larm) (/ 180 pi) (/ -180 pi)))
	   angle))
  )


(defun get-x-distance-to-door (arm &optional (real *real*))
  (let (current-lhand-angle)
    (dotimes (i 100)
      (send *aero* :upper arm :move-end-pos #f(10 0 0) :world)
      (send *irtviewer* :draw-objects)
      (cond ((eq real t)
	     (send *ri* :angle-vector (send *aero* :stroke-vector) 1000)
	     (unix::sleep (* 1000 1000))))
      (send *aero* :hand arm :open-pose)
      (cond ((eq real t)
	     (send *ri* :angle-vector (send *aero* :stroke-vector) 3000)
	     (unix::sleep (* 3000 1000))
	     (setq current-lhand-angle (if (eq arm :larm)
					   (elt (send *ri* :actual-vector) 7)
					 (elt (send *ri* :actual-vector) 18)))
	     (if (> (abs (- current-lhand-angle (send *aero* :upper arm :t-1p))) 60)
		 (progn (send *aero* :hand arm :close-pose)
			(send *ri* :angle-vector (send *aero* :stroke-vector) 1000)
			(unix::sleep (* 1000 1000))
			(return-from get-x-distance-to-door (* (- i 1) 10))))
	     )
	    (t
	     (send *aero* :hand arm :joint-angle (if (eq arm :larm) 40 -40))
	     (if (not (eq (send *aero-upper* :self-collision-check :pairs
				(list (cons (send *aero-upper* arm :f-1p :child-link)
					    (send *door* :link :door-panel))
				      (cons (send *aero-upper* arm :f-2p :child-link)
					    (send *door* :link :door-panel))
				      (cons (send *aero-upper* arm :f-3p :child-link)
					    (send *door* :link :door-panel)))) nil))
		 (progn (send *aero* :hand arm :close-pose)
			(return-from get-x-distance-to-door (* (- i 1) 10))))
	     )))
    ))

(defun zoom-grab ()
  (go? `(send *aero* :hand *arm* :joint-angle (if (eq *arm* :larm) -20 20)))
  (go? `(send *aero* :upper *arm* :move-end-pos (float-vector (+ *hand-free-space* (/ *knob-x* 2)) 0 0)) nil)
  (go? `(send *aero* :hand *arm* :joint-angle (if (eq *arm* :larm) 30 -30)))
  )

;; model samples

(defun sample-v1.00 ()
  (defvar *arm* :larm)
  (defvar *knob-x* 50)
  (defvar *knob-height* 745)
  (defvar *hand-free-space* 20)
  (send (send *door* :link :door-knob) :translate
	(float-vector 0 0 (- *knob-height* 845)))
  (go? `(setup-v1.00) nil)
  (go? `(hand-yaw-world *arm* (if (eq *arm* :larm) -90 90)))
  (go? `(get-x-distance-to-door *arm*) nil)
  (setq diff-to-door-knob (v- (send (send *door* :link :door-knob) :worldpos)
			      (send *aero* :upper *arm* :end-coords :worldpos)))
  (go? `(send *aero* :upper *arm* :move-end-pos
	      (float-vector (- (+ *knob-x* *hand-free-space*)) 0 0)))
  (go? `(send *aero* :hand *arm* :open-pose))
  (go? `(send *aero* :upper *arm* :elbow-p :joint-angle -120))
  (go? `(send *aero* :move-waist (float-vector 0 0 (aref diff-to-door-knob 2))))
  (go? `(send *aero* :upper *arm* :elbow-p :joint-angle -90))
  (zoom-grab)
  ))

(defun sample-beta ()
  ; set poses
  (defvar *rarm-avoid-pose*
    #f(40.1007 -29.388 4.91744 -71.6494 -22.7883 -2.70865 7.77037 15.0 0.0 0.0 0.0))
  (go? `(setup-beta) nil)
  (go? `(send *aero* :upper :rarm :angle-vector *rarm-avoid-pose*))
  ; test-drc-door-task takes a step-by-step fullbody-ik to grasp the knob
  ; memorizes the posture, and sends angle-vector at once for the real robot
  ; because aero's body is smaller than hrp2, we might need a different plan
  ; instead of trying ik, this code tries to reach as near the knob as possible 
  (setq diff-to-door-knob (v- (send (send
				     (send *door* :joint :door-knob-joint)
				     :child-link)
				    :worldpos)
			      (send *aero* :upper :larm :end-coords :worldpos))
	)
  (go? `(send *aero* :upper :reach-arm :larm :stop (abs (elt diff-to-door-knob 0))
	      :toward (float-vector 1 0 0) :adjust #f(0 0 -1)))
  (setq diff-to-door-knob (v- (send (send
				     (send *door* :joint :door-knob-joint)
				     :child-link)
				    :worldpos)
			      (send *aero* :upper :larm :end-coords :worldpos))
	)
  (go? `(send *aero* :upper :reach-arm :larm :stop (abs (elt diff-to-door-knob 1))
	      :toward (float-vector 0 1 0) :adjust #f(0 0 -1)))
  (setq diff-to-door-knob (v- (send (send
				     (send *door* :joint :door-knob-joint)
				     :child-link)
				    :worldpos)
			      (send *aero* :upper :larm :end-coords :worldpos))
	)
  (go? `(send *aero* :upper :reach-arm :larm :stop (abs (elt diff-to-door-knob 2))
	      :toward (float-vector 0 0 1) :adjust #f(-1 0 0)))
  (send (send *door* :joint :door-knob-joint) :joint-angle -55)
  (go? `(send *aero* :upper :larm :move-end-pos #f(0 0 -50)))
  (send (send *door* :joint :door-hinge-joint) :joint-angle -15)
  (go? `(send *aero* :upper :larm :move-end-pos #f(-150 -130 -30)))
  )
