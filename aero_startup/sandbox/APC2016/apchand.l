(defclass suction-finger
  :super cascaded-link
  :slots (fbase f0 f1 f2 jbase j0 j1))
(defmethod suction-finger
  (:init
   (&rest args)
   (send-super* :init args)
   (let (fbase-body f0-body f1-body f2-body)
     (setq fbase-body (make-cube 20 20 20))
     (send fbase-body :set-color :grey)

     (setq f0-body (make-cube 24 24 24))
     (send f0-body :set-color :red)

     (setq f1-body (make-cube 80 20 20))
     (send f1-body :translate #f(40 0 0))
     (send f1-body :set-color :green)

     (setq f2-body (make-cylinder 16 24))
     (send f2-body :translate #f(80 0 -24))
     (send f2-body :set-color :blue)

     (setq fbase (instance bodyset-link :init (make-cascoords)
			   :bodies (list fbase-body)))
     (setq f0 (instance bodyset-link :init (make-cascoords)
			:bodies (list f0-body)))
     (setq f1 (instance bodyset-link :init (make-cascoords)
			:bodies (list f1-body)))
     (setq f2 (instance bodyset-link :init (make-cascoords :pos #f(80 0 0))
			:bodies (list f2-body)))

     (setq jbase (instance rotational-joint :init
			:parent-link fbase :child-link f0
			:axis :x))
     (setq j0 (instance rotational-joint :init
			:parent-link f0 :child-link f1
			:axis :y))
     (setq j1 (instance rotational-joint :init
			:parent-link f1 :child-link f2
			:axis :y))

     (send self :assoc fbase)
     (send fbase :assoc f0)
     (send f0 :assoc f1)
     (send f1 :assoc f2)

     (setq links (list fbase f0 f1 f2))
     (setq joint-list (list jbase j0 j1))
     (send self :init-ending)
     )
   self
   )
  (:j0 (&rest args) (forward-message-to j0 args))
  (:j1 (&rest args) (forward-message-to j1 args))
  (:jbase (&rest args) (forward-message-to jbase args))
  )

(defun make-apc-finger () (instance suction-finger :init))

(setq *finger0* (make-apc-finger)
      *finger1* (make-apc-finger)
      *finger2* (make-apc-finger)
      *finger3* (make-apc-finger))
(send *finger0* :translate (float-vector 50 50 0))
(send *finger0* :rotate -pi/2 :y)
(send *finger0* :rotate pi :x)
(send *finger1* :translate (float-vector 50 -50 0))
(send *finger1* :rotate -pi/2 :y)
(send *finger1* :rotate pi :x)
(send *finger2* :translate (float-vector -50 50 0))
(send *finger2* :rotate -pi/2 :y)
(send *finger3* :translate (float-vector -50 -50 0))
(send *finger3* :rotate -pi/2 :y)
(setq *fingers* (list *finger0* *finger1* *finger2* *finger3*))

(setq *hand-base* (make-cube 120 120 60))
(send *hand-base* :translate (float-vector 0 0 -30))
(send *hand-base* :set-color :grey)
(dolist (finger *fingers*)
  (send *hand-base* :assoc finger))

(objects (list *finger0* *finger1* *finger2* *finger3* *hand-base*))


(defun pinch-mode ()
  (send-all *fingers* :jbase :joint-angle 0)
  (send-all *fingers* :j0 :joint-angle 0)
  (send-all *fingers* :j1 :joint-angle 0))

(defun power-mode ()
  (send *finger0* :jbase :joint-angle 45)
  (send *finger1* :jbase :joint-angle -45)
  (send *finger2* :jbase :joint-angle -45)
  (send *finger3* :jbase :joint-angle 45)
  (send-all *fingers* :j0 :joint-angle 0)
  (send-all *fingers* :j1 :joint-angle 0))

(defun vacuum-mode ()
  (send-all *fingers* :jbase :joint-angle 0)
  (send-all *fingers* :j0 :joint-angle 0)
  (send-all *fingers* :j1 :joint-angle -90))

(setq *ball* (make-gdome (make-gdome (make-icosahedron 50))))
(send *ball* :set-color :yellow)
(send *ball* :translate #f(0 0 80))

(setq *box* (make-cube 30 200 120))
(send *box* :set-color :yellow)
(send *box* :translate #f(0 0 100))

(setq *sheet* (make-cube 200 200 1))
(send *sheet* :set-color :yellow)
(send *sheet* :translate #f(0 0 106))

(defun pinch-grasp ()
  (pinch-mode)
  (send *irtviewer* :objects
	(list *finger0* *finger1* *finger2* *finger3* *hand-base* *box*))
  (send-all *fingers* :j0 :joint-angle (- 10))
  (send-all *fingers* :j1 :joint-angle 10)
  (send *irtviewer* :draw-objects)
  (unix:usleep (* 500 1000))
  (dotimes (i 20)
    (send-all *fingers* :j0 :joint-angle (- i 10))
    (send-all *fingers* :j1 :joint-angle (- 10 i))
    (send *irtviewer* :draw-objects)
    (unix:usleep (* 100 1000)))
  )

(defun power-grasp ()
  (power-mode)
  (send *irtviewer* :objects
	(list *finger0* *finger1* *finger2* *finger3* *hand-base* *ball*))
  (send-all *fingers* :j0 :joint-angle (- 20))
  (send-all *fingers* :j1 :joint-angle 20)
  (send *irtviewer* :draw-objects)
  (unix:usleep (* 500 1000))
  (dotimes (i 20)
    (send-all *fingers* :j0 :joint-angle (- i 20))
    (send-all *fingers* :j1 :joint-angle (- 20 i))
    (send *irtviewer* :draw-objects)
    (unix:usleep (* 100 1000)))
  )

(defun vacuum-grasp ()
  (vacuum-mode)
  (send *irtviewer* :objects
	(list *finger0* *finger1* *finger2* *finger3* *hand-base* *sheet*))
  (send *hand-base* :locate (float-vector 0 0 -50) :world)
  (send *irtviewer* :draw-objects)
  (unix:usleep (* 500 1000))
  (dotimes (i 20)
    (send *hand-base* :translate (float-vector 0 0 1))
    (send *irtviewer* :draw-objects)
    (unix:usleep (* 100 1000)))
  (send *hand-base* :locate (float-vector 0 0 -30) :world)
  )
