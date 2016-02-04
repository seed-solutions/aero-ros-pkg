(load "kiva-pod-meshes.l")
(defclass kiva-pod-model
  :super cascaded-link
  :slots (blink spot-a spot-b spot-c
		spot-d spot-e spot-f
		spot-g spot-h spot-i
		spot-j spot-k spot-l spots))
(defmethod kiva-pod-model
  (:init
   (&rest args &key (name "kiva-pod") &allow-other-keys)
   (let (bc)
     (send-super* :init :name name args)
     (setq bc (make-kiva-pod-faceset))
     (send (car bc) :set-color :gray)
     (setq blink (instance bodyset-link :init (make-cascoords) :bodies bc))


     (setq spot-a (make-cascoords :pos #f(250 -278 1550) :name "spot-a"))
     (setq spot-b (make-cascoords :pos #f(250 0 1550) :name "spot-b"))
     (setq spot-c (make-cascoords :pos #f(250 278 1550) :name "spot-c"))

     (setq spot-d (make-cascoords :pos #f(250 -278 1320) :name "spot-d"))
     (setq spot-e (make-cascoords :pos #f(250 0 1320) :name "spot-e"))
     (setq spot-f (make-cascoords :pos #f(250 278 1320) :name "spot-f"))

     (setq spot-g (make-cascoords :pos #f(250 -278 1090) :name "spot-g"))
     (setq spot-h (make-cascoords :pos #f(250 0 1090) :name "spot-h"))
     (setq spot-i (make-cascoords :pos #f(250 278 1090) :name "spot-i"))

     (setq spot-j (make-cascoords :pos #f(250 -278 830) :name "spot-j"))
     (setq spot-k (make-cascoords :pos #f(250 0 830) :name "spot-k"))
     (setq spot-l (make-cascoords :pos #f(250 278 830) :name "spot-l"))

     (setq spots (list spot-a spot-b spot-c
		       spot-d spot-e spot-f
		       spot-g spot-h spot-i
		       spot-j spot-k spot-l))

     (send self :assoc blink)
     (dolist (spot spots) (send self :assoc spot))

     (setq links (list blink))
     (setq joint-list nil)
     (send self :init-ending)
     )
   self
   )
  (:spot-a (&rest args) (forward-message-to spot-a args))
  (:spot-b (&rest args) (forward-message-to spot-b args))
  (:spot-c (&rest args) (forward-message-to spot-c args))
  (:spot-d (&rest args) (forward-message-to spot-d args))
  (:spot-e (&rest args) (forward-message-to spot-e args))
  (:spot-f (&rest args) (forward-message-to spot-f args))
  (:spot-g (&rest args) (forward-message-to spot-g args))
  (:spot-h (&rest args) (forward-message-to spot-h args))
  (:spot-i (&rest args) (forward-message-to spot-i args))
  (:spot-j (&rest args) (forward-message-to spot-j args))
  (:spot-k (&rest args) (forward-message-to spot-k args))
  (:spot-l (&rest args) (forward-message-to spot-l args))
  )

(defun make-kiva-pod ()
  (instance kiva-pod-model :init))
