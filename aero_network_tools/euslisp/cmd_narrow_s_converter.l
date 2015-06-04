#!/usr/bin/env roseus

(ros::load-ros-manifest "aero_network_tools")

;; (ros::load-ros-manifest "roseus")

;;;
(ros::roseus "cmd_narrow_s_converter")
(ros::advertise "/aero_network/cmd_narrow_band/streamer" aero_network_tools::AeroNarrowCommand 50)
(ros::advertise "/aero_network/ocs_av_narrow_band/streamer" aero_network_tools::AeroJointAngles 50)

(defvar *exit-flag* nil)

(defun av_narrow_receiver_cb (msg)
  ;; (print (send msg :angleVector))
  (send *aero* :angle-vector
        (setq angle-vector (list
                            (concatenate float-vector
                                         (subseq (send msg :angleVector) 0 8)
                                         #f(0 0 0)
                                         (subseq (send msg :angleVector) 8 16)
                                         #f(0 0 0)
                                         (subseq (send msg :angleVector) 16 22))
                            (concatenate float-vector
                                         (subseq (send msg :angleVector)  22 34)
                                         #f(0)))))
  (if (boundp '*irtviewer*)
      (send *irtviewer* :draw-objects))
  )

(ros::subscribe "/aero_network/ocs_av_narrow_band/receiver" aero_network_tools::AeroJointAngles #'av_narrow_receiver_cb)


(defun narrow-band-mode ()
  (while (ros::ok)
    (print "enter command")
    (if (eq *exit-flag* t) (return-from narrow-band-mode nil))
    (setq ~ui~ (read-line))
    (unless (eq (length ~ui~) 0)
      (if (equal ~ui~ "q")
          (return-from narrow-band-mode nil))
      (if (not (and (equal #\( (char ~ui~ 0))
                    (equal #\) (char ~ui~ (- (length ~ui~) 1)))))
          (setq ~ui~ (format nil "(~a)" ~ui~)))
      (convert-to-narrow-command (read-from-string ~ui~))
      )
    (ros::spin-once)
    (if (boundp '*irtviewer*)
        (send *irtviewer* :draw-objects))
    )
    (format t "ros::ok returned false~%")
    (format t "exit narrow-band-mode~%")
  )


(defun convert-to-narrow-command (cmd)
  (if (< (length cmd) 1)
      (return-from convert-to-narrow-command nil)) 

  (if (numberp (elt cmd 0))
      (progn
        (if (< (length cmd) 4)
            (progn
              (print "not enought arguments")
              (print "expected format: comand x y z (theta) (coords)")
              (return-from convert-to-narrow-command t)
              )
          (progn
            (setq theta 0)
            (setq coords 0)
            (if (> (length cmd) 4)
                (setq theta (elt cmd 4)))
            (if (> (length cmd) 5)
                (setq coords (elt cmd 5)))
            (ros::publish "/aero_network/cmd_narrow_band/streamer"
                          (instance aero_network_tools::AeroNarrowCommand :init
                                    :cmd (elt cmd 0)
                                    :x (elt cmd 1)
                                    :y (elt cmd 2)
                                    :z (elt cmd 3)
                                    :theta theta
                                    :coords coords
                                    ))
            (format t "send AeroNarrowCommand :cmd ~d :x ~d :y ~d :z ~d :theta ~f :coords ~d~%"
                    (elt cmd 0) (elt cmd 1) (elt cmd 2) (elt cmd 3) theta coords )
            (return-from convert-to-narrow-command t)
            )
          )
        )
    )

  
  (case (elt cmd 0)
	;; common
	(send ;; send
      (if (< (length cmd) 2)
          (progn
            (print "not enough arguments")
            (return-from convert-to-narrow-command nil)))
      (case (elt cmd 1)
        (*aero* ;; send *aero*
          (if (< (length cmd) 3)
          (progn
            (print "not enough arguments")
            (return-from convert-to-narrow-command nil)))
          (case (elt cmd 2)

            (:match-ri ;; send *aero* :match-ri
             (ros::publish "/aero_network/cmd_narrow_band/streamer"
                           (instance aero_network_tools::AeroNarrowCommand :init
                                     :cmd -126))
             (print "send *aero* :angle-vector")
              ) 

            (:angle-vector ;; send *aero* :angle-vector
             (setq av (send *aero* :angle-vector))
             (setq av (concatenate float-vector (elt av 0) (elt av 1)))
             (ros::publish "/aero_network/ocs_av_narrow_band/streamer"
                           (instance aero_network_tools::AeroJointAngles :init
                                     :angleVector (concatenate float-vector
                                                                (subseq av 0 8)
                                                                (subseq av 11 19)
                                                                (subseq av 22 41))))
             (print "send *aero* :angle-vector")
              )
            
            (:reset-manip-pose ;; send *aero* :reset-manip-pose
             (ros::publish "/aero_network/cmd_narrow_band/streamer"
                           (instance aero_network_tools::AeroNarrowCommand :init
                                     :cmd 16 :x 0))
             (send *aero* :reset-manip-pose)
             (print "send *aero* :reset-manip-pose")
              )
            
            (:reset-terrain-pose ;; send *aero* :reset-terrain-pose
             (ros::publish "/aero_network/cmd_narrow_band/streamer"
                           (instance aero_network_tools::AeroNarrowCommand :init
                                     :cmd 16 :x 1))
             (send *aero* :reset-terrain-pose)
             (print "send *aero* :reset-terrain-pose")
              )

            (:reset-terrain-pose2 ;; send *aero* :reset-terrain-pose2
             (ros::publish "/aero_network/cmd_narrow_band/streamer"
                           (instance aero_network_tools::AeroNarrowCommand :init
                                     :cmd 16 :x 2))
             (send *aero* :reset-terrain-pose2)
             (print "send *aero* :reset-terrain-pose2")
              )

            (:reset-climbing-pose ;; send *aero* :reset-climbing-pose
             (ros::publish "/aero_network/cmd_narrow_band/streamer"
                           (instance aero_network_tools::AeroNarrowCommand :init
                                     :cmd 16 :x 3))
             (send *aero* :reset-climbing-pose)
             (print "send *aero* :reset-climbing-pose")
              )

            (:reset-climbing-pose2 ;; send *aero* :reset-climbing-pose2
             (ros::publish "/aero_network/cmd_narrow_band/streamer"
                           (instance aero_network_tools::AeroNarrowCommand :init
                                     :cmd 16 :x 4))
             (send *aero* :reset-climbing-pose2)
             (print "send *aero* :reset-climbing-pose2")
              )
                        

            
            (:move-waist ;; send *aero* :move-waist
              (if (< (length cmd) 4)
                  (progn
                    (print "not enough arguments")
                    (return-from convert-to-narrow-command nil)))
              (let ((x (aref (elt cmd 3) 0))
                    (y (aref (elt cmd 3) 1))
                    (z (aref (elt cmd 3) 2)))
                (ros::publish "/aero_network/cmd_narrow_band/streamer"
                              (instance aero_network_tools::AeroNarrowCommand :init
                                        :cmd 8
                                        :x x
                                        :y y
                                        :z z))
                (send *aero* :move-waist (elt cmd 3))
                (format t "send *aero* :move-waist ~a~%" (elt cmd 3)))
              ) ;; end of (elt cmd 2) = :move-waist 

            (:go-pos ;; send *aero* :go-pos
              (if (< (length cmd) 6)
                  (progn
                    (print "not enough arguments")
                    (print "x y theta expected")
                    (return-from convert-to-narrow-command nil)))
              (let ((x (elt cmd 3))
                    (y (elt cmd 4))
                    (theta (elt cmd 5)))
                (if (< (length cmd) 7)
                   (progn 
                     (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                   (instance aero_network_tools::AeroNarrowCommand :init
                                             :cmd 10
                                             :x x
                                             :y y
                                             :theta theta
                                             :coords nil))
                     (send *aero* :go-pos x y theta)
                     (format t "send *aero* :go-pos ~f ~f ~f~%" (elt cmd 3) (elt cmd 4) (elt cmd 5))
                     )
                  (progn
                    (if (elt cmd 6)
                        (setq coords t)
                      (setq coords nil))
                    (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                  (instance aero_network_tools::AeroNarrowCommand :init
                                            :cmd 10
                                            :x x
                                            :y y
                                            :theta theta
                                            :coords coords))
                    (send *aero* :go-pos x y theta coords)
                    (format t "send *aero* :go-pos ~f ~f ~f ~s~%" (elt cmd 3) (elt cmd 4) (elt cmd 5) coords)
                    )
                  )
                )
              ) ;; end of (elt cmd 2) = :go-pos 
            
            (:hand ;; send *aero* :hand
              (if (< (length cmd) 4)
                  (progn
                    (print "not enough arguments")
                    (return-from convert-to-narrow-command nil)))             
              (case (elt cmd 3)
                (:arms ;; send *aero* :hand :arms
                  (if (< (length cmd) 5)
                      (progn
                        (print "not enough arguments")
                        (return-from convert-to-narrow-command nil)))
                  (case (elt cmd 4)
                    (:joint-angle ;; send *aero* :hand :arms :joint-angle
                      (if (< (length cmd) 6)
                          (progn
                            (print "not enough arguments")
                            (return-from convert-to-narrow-command nil)))
                      (let ((theta (elt cmd 5)))
                        (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                      (instance aero_network_tools::AeroNarrowCommand :init
                                                :cmd -7
                                                :theta theta))
                        (send *aero* :hand :arms :joint-angle theta)
                        (format t "send *aero* :hand :arms :joint-angle ~f~%" theta)
                        )
                      ) ;; end of case (elt cmd 4) = :joint-angle
                    (:open-pose ;; send *aero* :hand :arms :open-pose
                      (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                    (instance aero_network_tools::AeroNarrowCommand :init
                                              :cmd -5))
                      (send *aero* :hand :arms :open-pose)
                      (print "send *aero* :hand :arms :open-pose")
                      ) ;; end of case (elt cmd 4) = :open-pose
                    (:close-pose ;; send *aero* :hand :arms :close-pose
                      (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                    (instance aero_network_tools::AeroNarrowCommand :init
                                              :cmd -3))
                      (send *aero* :hand :arms :close-pose)
                      (print "send *aero* :hand :arms :close-pose")
                      ) ;; end of case (elt cmd 4) = :close-pose
                    (otherwise
                     (print "unrecognized command!"))
                    ) ;; end of case (elt cmd 4)
                  ) ;; end of case (elt cmd 3) = :arms
                (:rarm ;; send *aero* :hand :rarm
                  (if (< (length cmd) 5)
                      (progn
                        (print "not enough arguments")
                        (return-from convert-to-narrow-command nil)))
                  (case (elt cmd 4)
                    (:close-pose ;; send *aero* :hand :rarm :close-pose
                      (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                    (instance aero_network_tools::AeroNarrowCommand :init
                                              :cmd 2))
                      (send *aero* :hand :rarm :close-pose)
                      (format t "send *aero* :hand :rarm :close-pose~%")
                      ) ;; end of case (elt cmd 4) = :close-pose
                    (:open-pose ;; send *aero* :hand :rarm :open-pose
                      (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                    (instance aero_network_tools::AeroNarrowCommand :init
                                              :cmd 4))
                      (send *aero* :hand :rarm :open-pose)
                      (print "send *aero* :hand :rarm :open-pose")
                      ) ;; end of case (elt cmd 4) = :open-pose
                    (:joint-angle ;; send *aero* :hand :rarm :joint-angle
                      (if (< (length cmd) 6)
                          (progn
                            (print "not enough arguments")
                            (return-from convert-to-narrow-command nil)))
                      (let ((theta (elt cmd 5)))
                        (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                      (instance aero_network_tools::AeroNarrowCommand :init
                                                :cmd 6
                                                :theta theta))
                        (send *aero* :hand :rarm :joint-angle theta)
                        (format t "send *aero* :hand :rarm :joint-angle ~f~%" theta)
                        )
                      ) ;; end of case (elt cmd 4) = :joint-angle
                    (otherwise
                     (print "unrecognized command!"))
                    ) ;; end of case (elt cmd 4)
                  ) ;; end of case (elt cmd 3) = :rarm
                (:larm ;; send *aero* :hand :larm
                  (if (< (length cmd) 5)
                      (progn
                        (print "not enough arguments")
                        (return-from convert-to-narrow-command nil)))
                  (case (elt cmd 4)
                    (:close-pose ;; send *aero* :hand :larm :close-pose
                      (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                    (instance aero_network_tools::AeroNarrowCommand :init
                                              :cmd 3))
                      (send *aero* :hand :larm :close-pose)
                      (format t "send *aero* :hand :larm :close-pose~%")
                      ) ;; end of case (elt cmd 4) = :close-pose
                    (:open-pose ;; send *aero* :hand :larm :open-pose
                      (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                    (instance aero_network_tools::AeroNarrowCommand :init
                                              :cmd 5))
                      (send *aero* :hand :larm :open-pose)
                      (print "send *aero* :hand :larm :open-pose")
                      ) ;; end of case (elt cmd 4) = :open-pose
                    (:joint-angle ;; send *aero* :hand :larm :joint-angle
                      (if (< (length cmd) 6)
                          (progn
                            (print "not enough arguments")
                            (return-from convert-to-narrow-command nil)))
                      (let ((theta (elt cmd 5)))
                        (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                      (instance aero_network_tools::AeroNarrowCommand :init
                                                :cmd 7
                                                :theta theta))
                        (send *aero* :hand :larm :joint-angle theta)
                        (format t "send *aero* :hand :larm :joint-angle ~f~%" theta)
                        )
                      ) ;; end of case (elt cmd 4) = :joint-angle
                    (otherwise
                     (print "unrecognized command!"))
                    ) ;; end of case (elt cmd 4)
                  ) ;; end of case (elt cmd 3) = :larm
                (otherwise
                 (print "unrecognized command!"))
                ) ;; end of case (elt cmd 3)
              ) ;; end of case (elt cmd 2) = :hand 



            (:upper ;; send *aero* :upper
              (if (< (length cmd) 4)
                  (progn
                    (print "not enough arguments")
                    (return-from convert-to-narrow-command nil)))             
              (case (elt cmd 3)
                (:arms ;; send *aero* :upper :arms
                  (if (< (length cmd) 5)
                      (progn
                        (print "not enough arguments")
                        (return-from convert-to-narrow-command nil)))
                  (case (elt cmd 4)
                    (:move-end-pos ;; send *aero* :upper :arms :move-end-pos
                      (if (< (length cmd) 6)
                          (progn
                            (print "not enough arguments")
                            (return-from convert-to-narrow-command nil)))
                      (let ((x (aref (elt cmd 5) 0))
                            (y (aref (elt cmd 5) 1))
                            (z (aref (elt cmd 5) 2)))
                        (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                      (instance aero_network_tools::AeroNarrowCommand :init
                                                :cmd -1
                                                :x x
                                                :y y
                                                :z z))
                        (send *aero* :upper :arms :move-end-pos (elt cmd 5))
                        (format t "send *aero* :upper :arms :move-end-pos ~a~%" (elt cmd 5)))
                      ) ;; end of case (elt cmd 4) = :move-end-pos
                    (otherwise
                     (print "unrecognized command!"))
                    ) ;; end of case (elt cmd 4)
                  ) ;; end of case (elt cmd 3) = :arms
                (:rarm ;; send *aero* :upper :rarm
                  (if (< (length cmd) 5)
                      (progn
                        (print "not enough arguments")
                        (return-from convert-to-narrow-command nil)))
                  (case (elt cmd 4)
                    (:move-end-pos ;; send *aero* :upper :rarm :move-end-pos
                      (if (< (length cmd) 6)
                          (progn
                            (print "not enough arguments")
                            (return-from convert-to-narrow-command nil)))
                      (let ((x (aref (elt cmd 5) 0))
                            (y (aref (elt cmd 5) 1))
                            (z (aref (elt cmd 5) 2)))
                        (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                      (instance aero_network_tools::AeroNarrowCommand :init
                                                :cmd 0
                                                :x x
                                                :y y
                                                :z z))
                        (send *aero* :upper :rarm :move-end-pos (elt cmd 5))
                        (format t "send *aero* :upper :rarm :move-end-pos ~a~%" (elt cmd 5)))
                      ) ;; end of case (elt cmd 4) = :move-end-pos
                    (:move-end-rot ;; send *aero* :upper :rarm :move-end-rot
                      (if (< (length cmd) 7)
                          (progn
                            (print "not enough arguments")
                            (return-from convert-to-narrow-command nil)))
                      (let ((x (aref (elt cmd 5) 0))
                            (y (aref (elt cmd 5) 1))
                            (z (aref (elt cmd 5) 2))
                            (theta (rad2deg (elt cmd 6))))
                        (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                      (instance aero_network_tools::AeroNarrowCommand :init
                                                :cmd 14
                                                :x x
                                                :y y
                                                :z z
                                                :theta theta))
                        (send *aero* :upper :rarm :move-end-rot (elt cmd 5) (elt cmd 6))
                        (format t "send *aero* :upper :rarm :move-end-rot ~a~%" (elt cmd 5) (elt cmd 6)))
                      ) ;; end of case (elt cmd 4) = :move-end-rot
                    (otherwise
                     (print "unrecognized command!"))
                    ) ;; end of case (elt cmd 4)
                  ) ;; end of case (elt cmd 3) = :rarm
                (:larm ;; send *aero* :upper :larm
                  (if (< (length cmd) 5)
                      (progn
                        (print "not enough arguments")
                        (return-from convert-to-narrow-command nil)))
                  (case (elt cmd 4)
                    (:move-end-pos ;; send *aero* :upper :larm :move-end-pos
                      (if (< (length cmd) 6)
                          (progn
                            (print "not enough arguments")
                            (return-from convert-to-narrow-command nil)))
                      (let ((x (aref (elt cmd 5) 0))
                            (y (aref (elt cmd 5) 1))
                            (z (aref (elt cmd 5) 2)))
                        (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                      (instance aero_network_tools::AeroNarrowCommand :init
                                                :cmd 1
                                                :x x
                                                :y y
                                                :z z))
                        (send *aero* :upper :larm :move-end-pos (elt cmd 5))
                        (format t "send *aero* :upper :larm :move-end-pos ~a~%" (elt cmd 5)))
                      ) ;; end of case (elt cmd 4) = :move-end-pos
                    (:move-end-rot ;; send *aero* :upper :larm :move-end-rot
                      (if (< (length cmd) 7)
                          (progn
                            (print "not enough arguments")
                            (return-from convert-to-narrow-command nil)))
                      (let ((x (aref (elt cmd 5) 0))
                            (y (aref (elt cmd 5) 1))
                            (z (aref (elt cmd 5) 2))
                            (theta (deg2rad (elt cmd 6))))
                        (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                      (instance aero_network_tools::AeroNarrowCommand :init
                                                :cmd 15
                                                :x x
                                                :y y
                                                :z z
                                                :theta theta))
                        (send *aero* :upper :larm :move-end-rot (elt cmd 5) (elt cmd 6))
                        (format t "send *aero* :upper :larm :move-end-rot ~a~%" (elt cmd 5) (elt cmd 6)))
                      ) ;; end of case (elt cmd 4) = :move-end-rot
                    (otherwise
                     (print "unrecognized command!"))
                    ) ;; end of case (elt cmd 4)
                  ) ;; end of case (elt cmd 3) = :larm
                (otherwise
                 (print "unrecognized command!"))
                ) ;; end of case (elt cmd 3)
              ) ;; end of case (elt cmd 2) = :upper 


            (:rotate-knob ;; send *aero* :rotate-knob
              (if (< (length cmd) 4)
                  (progn
                    (print "not enough arguments")
                    (return-from convert-to-narrow-command nil)))             
              (case (elt cmd 3)
                (:larm ;; send *aero* :rotate-knob :larm
                  (if (< (length cmd) 5)
                      (progn
                        (print "not enough arguments")
                        (return-from convert-to-narrow-command nil)))
                  (let ((y (elt cmd 5)))
                        (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                      (instance aero_network_tools::AeroNarrowCommand :init
                                                :cmd 41
                                                :y y))
                        (send *aero* :rotate-knob :larm y)
                        (format t "send *aero* :rotate-knob :larm ~f~%" y)
                        )
                  ) ;; end of case (elt cmd 3) = :larm
                (otherwise
                 (print "unrecognized command!"))
                ) ;; end of case (elt cmd 3)
              ) ;; end of case (elt cmd 2) = :rotate-knob

            (:open-door ;; send *aero* :open-door
              (if (< (length cmd) 4)
                  (progn
                    (print "not enough arguments")
                    (return-from convert-to-narrow-command nil)))             
              (case (elt cmd 3)
                (:larm ;; send *aero* :open-door :larm
                  (if (< (length cmd) 5)
                      (progn
                        (print "not enough arguments")
                        (return-from convert-to-narrow-command nil)))
                  (let ((z (elt cmd 5)))
                        (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                      (instance aero_network_tools::AeroNarrowCommand :init
                                                :cmd 43
                                                :z z))
                        (send *aero* :open-door :larm z)
                        (format t "send *aero* :open-door :larm ~f~%" z)
                        )
                  ) ;; end of case (elt cmd 3) = :larm
                (otherwise
                 (print "unrecognized command!"))
                ) ;; end of case (elt cmd 3)
              ) ;; end of case (elt cmd 2) = :open-door

            (:rotate-valve ;; send *aero* :rotate-valve
              (if (< (length cmd) 4)
                  (progn
                    (print "not enough arguments")
                    (return-from convert-to-narrow-command nil)))             
              (case (elt cmd 3)
                (:larm ;; send *aero* :rotate-valve :larm
                  (if (< (length cmd) 6)
                      (progn
                        (print "not enough arguments")
                        (return-from convert-to-narrow-command nil)))
                  (let ((x (elt cmd 5))
                        (theta (elt cmd 6))
                        (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                      (instance aero_network_tools::AeroNarrowCommand :init
                                                :cmd 51
                                                :x x
                                                :theta theta))
                        (send *aero* :rotate-valve :larm x theta)
                        (format t "send *aero* :rotate-valve :larm ~f ~f~%" x theta)
                        )
                    )
                  ) ;; end of case (elt cmd 3) = :larm
                (:rarm ;; send *aero* :rotate-valve :rarm
                  (if (< (length cmd) 6)
                      (progn
                        (print "not enough arguments")
                        (return-from convert-to-narrow-command nil)))
                  (let ((x (elt cmd 5))
                        (theta (elt cmd 6))
                        (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                      (instance aero_network_tools::AeroNarrowCommand :init
                                                :cmd 50
                                                :x x
                                                :theta theta))
                        (send *aero* :rotate-valve :rarm x theta)
                        (format t "send *aero* :rotate-valve :rarm ~f ~f~%" x theta)
                        )
                    )
                  ) ;; end of case (elt cmd 3) = :rarm
                (otherwise
                 (print "unrecognized command!"))
                ) ;; end of case (elt cmd 3)
              ) ;; end of case (elt cmd 2) = :open-door

            (:release-arm ;; send *aero* :release-arm
              (if (< (length cmd) 4)
                  (progn
                    (print "not enough arguments")
                    (return-from convert-to-narrow-command nil)))             
              (case (elt cmd 3)
                (:larm ;; send *aero* :release-arm :larm
                 (ros::publish "/aero_network/cmd_narrow_band/streamer"
                               (instance aero_network_tools::AeroNarrowCommand :init
                                         :cmd 53))
                        (send *aero* :release-arm :larm)
                        (format t "send *aero* :release-arm :larm~%")
                  ) ;; end of case (elt cmd 3) = :larm
                (:rarm ;; send *aero* :release-arm :rarm
                 (ros::publish "/aero_network/cmd_narrow_band/streamer"
                               (instance aero_network_tools::AeroNarrowCommand :init
                                         :cmd 52))
                        (send *aero* :release-arm :rarm)
                        (format t "send *aero* :release-arm :rarm~%")
                  ) ;; end of case (elt cmd 3) = :rarm
                (otherwise
                 (print "unrecognized command!"))
                ) ;; end of case (elt cmd 3)
              ) ;; end of case (elt cmd 2) = :release-arm

            (:magic-close ;; send *aero* :magic-close
              (if (< (length cmd) 4)
                  (progn
                    (print "not enough arguments")
                    (return-from convert-to-narrow-command nil)))             
              (case (elt cmd 3)
                (:larm ;; send *aero* :magic-close :larm
                 (ros::publish "/aero_network/cmd_narrow_band/streamer"
                               (instance aero_network_tools::AeroNarrowCommand :init
                                         :cmd 103))
                        (send *aero* :magic-close :larm)
                        (format t "send *aero* :magic-close :larm~%")
                  ) ;; end of case (elt cmd 3) = :larm
                (:rarm ;; send *aero* :magic-close :rarm
                 (ros::publish "/aero_network/cmd_narrow_band/streamer"
                               (instance aero_network_tools::AeroNarrowCommand :init
                                         :cmd 102))
                        (send *aero* :magic-close :rarm)
                        (format t "send *aero* :magic-close :rarm~%")
                  ) ;; end of case (elt cmd 3) = :rarm
                (otherwise
                 (print "unrecognized command!"))
                ) ;; end of case (elt cmd 3)
              ) ;; end of case (elt cmd 2) = :magic-close

            (:magic-open ;; send *aero* :magic-open
              (if (< (length cmd) 4)
                  (progn
                    (print "not enough arguments")
                    (return-from convert-to-narrow-command nil)))             
              (case (elt cmd 3)
                (:larm ;; send *aero* :magic-open :larm
                 (ros::publish "/aero_network/cmd_narrow_band/streamer"
                               (instance aero_network_tools::AeroNarrowCommand :init
                                         :cmd 105))
                        (send *aero* :magic-open :larm)
                        (format t "send *aero* :magic-open :larm~%")
                  ) ;; end of case (elt cmd 3) = :larm
                (:rarm ;; send *aero* :magic-open :rarm
                 (ros::publish "/aero_network/cmd_narrow_band/streamer"
                               (instance aero_network_tools::AeroNarrowCommand :init
                                         :cmd 104))
                        (send *aero* :magic-open :rarm)
                        (format t "send *aero* :magic-open :rarm~%")
                  ) ;; end of case (elt cmd 3) = :rarm
                (otherwise
                 (print "unrecognized command!"))
                ) ;; end of case (elt cmd 3)
              ) ;; end of case (elt cmd 2) = :magic-open

            (:hand-yaw-world ;; send *aero* :hand-yaw-world
              (if (< (length cmd) 4)
                  (progn
                    (print "not enough arguments")
                    (return-from convert-to-narrow-command nil)))             
              (case (elt cmd 3)
                (:larm ;; send *aero* :hand-yaw-world :larm
                  (if (< (length cmd) 5)
                      (progn
                        (print "not enough arguments")
                        (return-from convert-to-narrow-command nil)))
                  (let ((theta (elt cmd 5)))
                        (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                      (instance aero_network_tools::AeroNarrowCommand :init
                                                :cmd 107
                                                :theta theta))
                        (send *aero* :hand-yaw-world :larm theta)
                        (format t "send *aero* :hand-yaw-world :larm ~f~%" theta)
                        )
                  ) ;; end of case (elt cmd 3) = :larm
                (:rarm ;; send *aero* :hand-yaw-world :rarm
                  (if (< (length cmd) 5)
                      (progn
                        (print "not enough arguments")
                        (return-from convert-to-narrow-command nil)))
                  (let ((theta (elt cmd 5)))
                        (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                      (instance aero_network_tools::AeroNarrowCommand :init
                                                :cmd 106
                                                :theta theta))
                        (send *aero* :hand-yaw-world :rarm theta)
                        (format t "send *aero* :hand-yaw-world :rarm ~f~%" theta)
                        )
                  ) ;; end of case (elt cmd 3) = :rarm
                (otherwise
                 (print "unrecognized command!"))
                ) ;; end of case (elt cmd 3)
              ) ;; end of case (elt cmd 2) = :open-door
            
            
            (otherwise
             (print "unrecognized command!"))
            ) ;; end of case (elt cmd 2)
          ) ;; end of case (elt cmd 1) = *aero*




        (*walker* ;; send walker
          (if (< (length cmd) 3)
          (progn
            (print "not enough arguments")
            (return-from convert-to-narrow-command nil)))
          (case (elt cmd 2)
            (:step ;; send walker :step
              (if (< (length cmd) 5)
                  (progn
                    (print "not enough arguments")
                    (return-from convert-to-narrow-command nil)))
              (let ((x (aref (elt cmd 3) 0))
                    (y (aref (elt cmd 3) 1))
                    (z (aref (elt cmd 3) 2)))
                
                (case (elt cmd 4)
                      (:rarm
                       (setq cmdCode 61))
                      (:larm
                       (setq cmdCode 62))
                      (:lleg
                       (setq cmdCode 63))
                      (:rleg
                       (setq cmdCode 64))
                      (otherwise
                       (print "expecting limb as 5th argument")
                       (return-from convert-to-narrow-command nil))
                      )
                (if (position ':balance-rate-thre cmd)
                    (setq balance-rate-thre (elt cmd (+ 1 (position ':balance-rate-thre cmd))))
                  (setq balance-rate-thre (send *walker* :get-val `def_balance_rate_thre)))
                (if (position ':step-height cmd)
                    (setq step-height (elt cmd (+ 1 (position ':step-height cmd))))
                  (setq step-height (send *walker* :get-val `def_step_height)))
                
                (ros::publish "/aero_network/cmd_narrow_band/streamer"
                              (instance aero_network_tools::AeroNarrowCommand :init
                                        :cmd cmdCode
                                        :x x
                                        :y y
                                        :z z
                                        :theta balance-rate-thre
                                        :coords step-height))
                (send *walker* :step (elt cmd 3) (elt cmd 4) :balance-rate-thre balance-rate-thre :step-height step-height)
                (format t "send *walker* :step ~a ~s :balance-rate-thre ~f :step-height ~d~%" (elt cmd 3) (elt cmd 4) balance-rate-thre step-height))
              ) ;; end of (elt cmd 2) = :step

            (:step-square ;; send walker :step-square
              (if (< (length cmd) 5)
                  (progn
                    (print "not enough arguments")
                    (return-from convert-to-narrow-command nil)))
              (let ((x (aref (elt cmd 3) 0))
                    (y (aref (elt cmd 3) 1))
                    (z (aref (elt cmd 3) 2)))
                
                (case (elt cmd 4)
                      (:rarm
                       (setq cmdCode 65))
                      (:larm
                       (setq cmdCode 66))
                      (:lleg
                       (setq cmdCode 67))
                      (:rleg
                       (setq cmdCode 68))
                      (otherwise
                       (print "expecting limb as 5th argument")
                       (return-from convert-to-narrow-command nil))
                      )
                (if (position ':balance-rate-thre cmd)
                    (setq balance-rate-thre (elt cmd (+ 1 (position ':balance-rate-thre cmd))))
                  (setq balance-rate-thre (send *walker* :get-val `def_balance_rate_thre)))
                (if (position ':step-height cmd)
                    (setq step-height (elt cmd (+ 1 (position ':step-height cmd))))
                  (setq step-height (send *walker* :get-val `def_step_square_height)))
                
                (ros::publish "/aero_network/cmd_narrow_band/streamer"
                              (instance aero_network_tools::AeroNarrowCommand :init
                                        :cmd cmdCode
                                        :x x
                                        :y y
                                        :z z
                                        :theta balance-rate-thre
                                        :coords step-height))
                (send *walker* :step-square (elt cmd 3) (elt cmd 4) :balance-rate-thre balance-rate-thre :step-height step-height)
                (format t "send *walker* :step-square ~a ~s :balance-rate-thre ~f :step-height ~d~%" (elt cmd 3) (elt cmd 4) balance-rate-thre step-height))
              ) ;; end of (elt cmd 2) = :step-square


            (:walk ;; send walker :walk                
              (if (position ':goal cmd)
                  (setq goal (elt cmd (+ 1 (position ':goal cmd))))
                (setq goal (send *walker* :get-val `def_walk_goal)))
              (setq x (aref goal 0))
              (setq y (aref goal 1))
              (if (position ':stride-length cmd)
                  (setq stride-length (elt cmd (+ 1 (position ':stride-length cmd))))
                (setq stride-length (send *walker* :get-val `def_stride_length)))
              (if (position ':step-height cmd)
                  (setq step-height (elt cmd (+ 1 (position ':step-height cmd))))
                (setq step-height (send *walker* :get-val `def_step_height)))
              (if (position ':balance-rate-thre cmd)
                  (setq balance-rate-thre (elt cmd (+ 1 (position ':balance-rate-thre cmd))))
                (setq balance-rate-thre (send *walker* :get-val `def_balance_rate_thre)))
              
              (ros::publish "/aero_network/cmd_narrow_band/streamer"
                            (instance aero_network_tools::AeroNarrowCommand :init
                                      :cmd 69
                                      :x x
                                      :y y
                                      :z step-height
                                      :theta balance-rate-thre
                                      :coords stride-length))
              (send *walker* :walk :goal (float-vector x y 0) :balance-rate-thre balance-rate-thre :step-height step-height :stride-length stride-length)
              (format t "send *walker* :walk :goal #f(~f ~f 0) :balance-rate-thre ~f :step-height ~d :stride-length ~d~%" x y balance-rate-thre step-height stride-length)
              ) ;; end of (elt cmd 2) = :walk
            
            (otherwise
             (print "unrecognized command!"))
            ) ;; end of case (elt cmd 2)
          ) ;; end of case (elt cmd 1) = walker
        
        
        
      

        
        (otherwise
         (print "unrecognized command!"))
        ) ;; end of case (elt cmd 1)
      ) ;; end of case (elt cmd 0) = send 
    (otherwise
     (print "unrecognized command!"))
    ) ;; end of case (elt cmd 0)
  )

