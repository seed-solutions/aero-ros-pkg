#!/usr/bin/env roseus

(ros::load-ros-manifest "aero_network_tools")

;; (ros::load-ros-manifest "roseus")

;;;
(ros::roseus "cmd_narrow_s_converter")
(ros::advertise "/aero_network/cmd_narrow_band/streamer" aero_network_tools::AeroNarrowCommand 50)

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

(ros::subscribe "/aero_network/av_narrow_band/receiver" aero_network_tools::AeroJointAngles #'av_narrow_receiver_cb)


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
    )
    (format t "ros::ok returned false~%")
    (format t "exit narrow-band-mode~%")
  )


(defun convert-to-narrow-command (cmd)
  (if (< (length cmd) 1)
      (return-from convert-to-narrow-command nil)) 
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
                        (send *aero* :hand :arms :joint-angle theta))
                      ) ;; end of case (elt cmd 4) = :joint-angle
                    (:open-pose ;; send *aero* :hand :arms :open-pose
                      (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                    (instance aero_network_tools::AeroNarrowCommand :init
                                              :cmd -5))
                      (send *aero* :hand :arms :open-pose)
                      ) ;; end of case (elt cmd 4) = :open-pose
                    (:close-pose ;; send *aero* :hand :arms :close-pose
                      (ros::publish "/aero_network/cmd_narrow_band/streamer"
                                    (instance aero_network_tools::AeroNarrowCommand :init
                                              :cmd -3))
                      (send *aero* :hand :arms :close-pose)
                      ) ;; end of case (elt cmd 4) = :close-pose
                    (otherwise
                     (print "unrecognized command!"))
                    ) ;; end of case (elt cmd 4)
                  ) ;; end of case (elt cmd 3) = :arms
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
                        (send *aero* :hand :arms :move-end-pos (elt cmd 5))
                        (format t "send *aero* :hand :arms :move-end-pos ~a~%" (elt cmd 5)))
                      ) ;; end of case (elt cmd 4) = :move-end-pos
                    (otherwise
                     (print "unrecognized command!"))
                    ) ;; end of case (elt cmd 4)
                  ) ;; end of case (elt cmd 3) = :arms
                (otherwise
                 (print "unrecognized command!"))
                ) ;; end of case (elt cmd 3)
              ) ;; end of case (elt cmd 2) = :hand 


            

            
            (otherwise
             (print "unrecognized command!"))
            ) ;; end of case (elt cmd 2)
          ) ;; end of case (elt cmd 1) = *aero*
        (otherwise
         (print "unrecognized command!"))
        ) ;; end of case (elt cmd 1)
      ) ;; end of case (elt cmd 0) = send 
    (otherwise
     (print "unrecognized command!"))
    ) ;; end of case (elt cmd 0)
  )

