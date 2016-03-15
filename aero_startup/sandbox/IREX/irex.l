#!/usr/bin/env roseus

(ros::load-ros-manifest "aero_startup")
(ros::roseus "irex_demo")

(load "package://aeroeus/aero-interface.l")
(load-controllers)
;; (load "package://aeroeus/pre-release/inverse-kinematics-extension.l")
(load "inverse-kinematics-extension.l")
(load "package://aero_teleop/euslisp/vi-teleop.l")

(ros::wait-for-service "/extract_object_features/perception_area")
(ros::wait-for-service "/color_memorizer/perception_area")
(ros::wait-for-service "/color_memorizer/find_match")

(defvar *timer* (instance mtimer :init))

(defvar *manip-area* (list '(-0.3 0.3) '(-0.3 0.3) '(0.0 1.0)) ) ;; x y z
(defvar *samples* (list (list "green" '(46 97) '(42 194) '(106 183)) ;; h s i
			(list "orange" '(-10 21) '(100 255) '(83 255))
			(list "blue" '(-128 -107) '(35 110) '(120 255))
			))

(defvar *null-area* (list '(0.0 0.0) '(0.0 0.0) '(0.0 0.0)))
(defvar *null-color* (list "null" '(0 0) '(0 0) '(0 0)))

(load "motions.l")
(load "nav.l")
(load "recog.l")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun init-irex-demo ()
  ;; initate demo
  (start-pose 5000)
  (setf *tl* (instance ros::transform-listener :init))
  (find-object *null-color* *null-area*)
  )

(defun set-object-to-search ()
  (let (color-tag)
    ;; set object to search
    (setq color-tag (set-object :timeout 60))
    (when (< color-tag 0)
      (format t "failed distinguishing object")
      (nod-no 500)
      (return-from demo nil)
      ) ;; failed distinguishing object
    (nod-yes 500)
    (elt *samples* color-tag)
    ))

(defun stand-to-work-station (color)
  (let (search-area)
    ;; ready for manipulation
    (run-pose 500)

    ;; search for object
    (setq search-area *manip-area*)
    (find-object color search-area)))

;; rotation only
(defun rotate-to-work-station (color)
  (when *real*
    (send *ri* :go-pos 0 0 55)
    (unix::usleep (* 5000 1000))
    )
  (stand-to-work-station color))
(defun rotate-to-original-station ()
  (when *real*
    (send *ri* :go-pos 0 0 -55)
    (unix::usleep (* 5000 1000))
    ))

(defun move-to-work-station (color)
  (let (search-area found-object search-flag m)
    ;; move base to face work station
    (when *real*
      (send *ri* :go-pos 0 0 110)
      (unix::usleep (* 8000 1000))
      )

    (setq found-object (stand-to-work-station color))
    (setq search-flag t)
    (while (and (< (cadr (elt search-area 2)) 5.0) ;; search area limit
                (not (> found-object 0))
                search-flag) ;; object not found
      (setq search-area
            (list (list -1.0 1.0)
                  (list -1.0 1.0)
                  (list 0.0 (+ (cadr (elt search-area 2)) 0.2))
                  )) ;; expand search-area
      (setq found-object (find-object color search-area))
      ;; re-initiate *tl* so that it does not find ghost tfs
      (setf *tl* (instance ros::transform-listener :init))
      ;; maybe tf was bad timing
      (unix::usleep (* 1000 1000))
      (setq m (send *tl* :lookup-transform
                    "leg_base_link" "object_l" (ros::time)))
      (when m
        (when (and
               (not (eps= (elt (send m :pos) 0) 0 0.1))
               (not (eps= (elt (send m :pos) 1) 0 0.1))
               (not (eps= (elt (send m :pos) 2) 0 0.1))
               (not (< (elt (send m :pos) 2) -500)) ;; stash floor
               (not (< (elt (send m :pos) 0) 1000)) ;; too close maybe noise
               )
          (setq search-flag nil)
          )
        ) ;; if the tf was published even once, then if would be t
      )

    (when (>= (cadr (elt search-area 2)) 5.0)
      (format t "failed finding actual object")
      (return-from move-to-work-station nil)
      ) ;; failed finding actual object

    ;; move base forward till robot is near object
    ;; go-for-target (go_x go_y) (until_x until_y)
    ;; if until_x > 999 then y will be ignored
    (setq go-went (go-for-target '(100000 0) '(0.72 1000)))
    (when (< (car go-went) 0)
      (format t "failed moving toward object")
      (return-from move-to-work-station nil)
      ) ;; failed moving toward object
    go-went
    ))

(defun move-to-sweet-spot ()
  (let (m y-direction go-sweet)
    ;; get to sweet manip position
    (setq m (send *tl* :lookup-transform
                  "leg_base_link" "object_l" (ros::time)))
    (when (or (< (elt (send m :pos) 1) 0)
              (> (elt (send m :pos) 1) 180))
      (cond ((< (elt (send m :pos) 1) 0) ;; object is on far right
             (setq y-direction "go-right")
             (setq go-sweet (go-for-target '(0 -1000) '(1000 0.35)))
             )
            ((> (elt (send m :pos) 1) 180) ;; object is on far left
             (setq y-direction "go-left")
             (setq go-sweet (go-for-target '(0 1000) '(1000 0.18)))
             )
            )
      (when (< (car go-sweet) 0)
        (format t "failed adjusting manip position")
        (return-from move-to-sweet-spot nil)
        ) ;; failed adjusting position
      )
    (list y-direction go-sweet)
    ))

(defun move-back-from-sweet-spot (y-direction go-sweet)
  ;; move back from sweet position
  (when go-sweet
    (if (equal y-direction "go-left") ;; moved direction
        (go-for-time 0 -1000 0 (cadr go-sweet))
      (go-for-time 0 1000 0 (cadr go-sweet))
      )
    )
  )

(defun move-to-original-station (go-went)
  ;; move base to face original station
  (when *real*
    (send *ri* :go-pos 0 0 -110)
    (unix::usleep (* 8000 1000))
    )
  ;; move base backward to starting point
  (go-for-time 100000 0 0 (+ (cadr go-went) 0.4))
  t
  )

(defun manipulate-at-work-station (color)
  (let (ik-err grasp-pose grasp-status waist-z move-time)
    ;; manipulate at work station
    (unix::usleep (* 3000 1000)) ;; for some reason move-waist is not called
    (when *real*
      (send *ri* :move-waist (float-vector 0 0 -200) :world)
      (unix::usleep (* 2200 1000))
      )
    (find-object color *manip-area*)
    (unix::usleep (* 2000 1000)) ;; has to wait for stable configure, 5000 for bad network
    ;; solve ik
    (setq ik-err (send *aero* :grasp-object :larm
                       :torso t :revert-if-fail nil :offset-z 50))
    (setq grasp-pose (send *aero* :angle-vector)) ;; save pose
    (setq waist-z (elt (cadr ik-err) 2)) ;; save height
    (when *real*
      (send *ri* :ungrasp :larm)
      (unix::usleep (* 1000 1000))
      (send *ri* :move-waist (cadr ik-err))
      (unix::usleep (* 1000 1000))
      (manip-pose 1000)
      (setq move-time 3000)
      (send *aero* :angle-vector grasp-pose)
      (when (send *aero* :larm :move-end-pos #f(-50 80 0)
                  :local :rotation-axis nil :traslation-axis :z)
        (send *ri* :angle-vector (send *aero* :angle-vector) 2000)
        (unix::usleep (* 2000 1000))
        (send *aero* :angle-vector grasp-pose)
        (setq move-time 1000)
        )
      (send *ri* :angle-vector (send *aero* :angle-vector) move-time)
      (unix::usleep (* move-time 1000))
      (setq grasp-status (send *ri* :grasp :larm))
      (unix::usleep (* 1000 1000))
      )
    (back-pose 5000)
    (list ik-err grasp-pose grasp-status)
    ))

(defun place-object (ik-err grasp-pose)
  ;; place object
  (unix::usleep (* 3000 1000)) ;; for some reason move-waist is not called
  (when *real*
    (send *ri* :move-waist (v+ #f(0 0 -200) (cadr ik-err)) :world)
    (unix::usleep (* 2200 1000))
    )
  (send *aero* :angle-vector grasp-pose)
  (send *aero* :head :neck-p :joint-angle 0)
  (send *aero* :head :neck-r :joint-angle 0)
  (send *aero* :head :neck-y :joint-angle 0)
  (when *real*
    (send *ri* :angle-vector (send *aero* :angle-vector) 3000)
    (unix::usleep (* 3000 1000))
    (send *ri* :ungrasp :larm)
    (unix::usleep (* 1000 1000))
    )
  ;; (end-pose 1000)
  (end-pose2 1000)
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun demo (&key (move nil) (adjust nil))
  (let (color-tag color
                  search-area found-object m search-flag
                  go-went (go-sweet nil) y-direction
                  ik-err grasp-pose grasp-status
                  )
    ;; initate demo
    (format t ";; move to initial position...~%")
    (init-irex-demo)

    ;; set object to search
    (format t ";; set target from photo, please let me see...~%")
    (setq color (set-object-to-search))
    (format t ";; found: ~A~%" (car color))

    ;; move base to work station
    (if move
        (progn
          (format t ";; move base to work station...~%")
          (setq go-went (move-to-work-station color))
          (unless go-went (return-from demo nil)))
      (progn
        (format t ";; stand to work station...~%")
        ;; (stand-to-work-station color)
        (rotate-to-work-station color)
        )
      )

    ;; get to sweet manip position
    (when adjust
      (format t ";; move to sweet spot...~%")
      (setq res (move-to-sweet-spot))
      (unless res
        (return-from demo nil))
      (setq y-direction (car res)
            go-sweet (cadr res)))

    ;; manipulate at work station
    (format t ";; pick up...~%")
    (setq res (manipulate-at-work-station color))
    (setq ik-err (elt res 0)
          grasp-pose (elt res 1)
          grasp-status (elt res 2))

    ;; move back from sweet position
    (when adjust
      (format t ";; move back from sweet spot...~%")
      (move-back-from-sweet-spot y-direction go-sweet))

    ;; move base to starting point
    (if move
        (progn
          (format t ";; move base to original station...~%")
          (move-to-original-station go-went))
      (progn
        (format t ";; stand to original station...~%")
        (rotate-to-original-station)
        )
      )

    ;; grasp failed
    (when (equal grasp-status "grasp failed")
      (format t ";; sorry, I could not grasp!~%")
      ;; (end-pose 500)
      (end-pose2 500)
      (sorry 1000)
      (start-pose 3000)
      (return-from demo nil)
      )

    ;; place object
    (format t ";; place object...~%")
    (place-object ik-err grasp-pose)
    ))

(aero-init)
(objects *aero*) ;; this has to be called or error occurs
(setq *real* t)
