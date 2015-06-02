#!/usr/bin/env roseus

(ros::load-ros-manifest "aero_network_tools")

;; (ros::load-ros-manifest "roseus")

;;;
(ros::roseus "cmd_narrow_s_converter")
(ros::advertise "/aero_network/command_narrow_band/streamer" aero_network_tools::AeroNarrowCommand 50)

(defvar *exit-flag* nil)

(defun av_narrow_receiver_cb (msg)
  ;; (print (send msg :angleVector))
  (setq angles (list (subseq (send msg :angleVector) 0 27) (subseq (send msg :angleVector)  28 40)))
  ;; (print angles)
  ;; (elt angles 0)
  (send *aero* :angle-vector (list (subseq (send msg :angleVector) 0 27) (subseq (send msg :angleVector)  28 40)))
  (if (boundp '*irtviewer*)
      (send *irtviewer* :draw-objects))
  )

(ros::subscribe "/aero_network_tools/av_narrow_band/receiver" aero_network_tools::AeroJointAngles #'av_narrow_receiver_cb)


(defun narrow-band-mode ()
  (while (ros::ok)
    (print "enter command")
    (if (eq *exit-flag* t) (return-from narrow-band-mode nil))
    (setq ~ui~ (read-line))
    (if (equal ~ui~ "q")
        (return-from narrow-band-mode nil))
    (if (not (and (equal #\( (char ~ui~ 0))
                  (equal #\) (char ~ui~ (- (length ~ui~) 1)))))
        (setq ~ui~ (format nil "(~a)" ~ui~)))
    (convert-to-narrow-command (read-from-string ~ui~))
    (ros::spin-once)
    )
  (format t "ros::ok returned false")
  )


(defun convert-to-narrow-command (cmd)
  (if (< (length cmd) 1)
      (return-from convert-to-narrow-command nil)) 
  (case (elt cmd 0)
	;; common
	(send ;; send
      (if (< (length cmd) 2)
          (return-from convert-to-narrow-command nil))
      (case (elt cmd 1)
        (*aero* ;; send *aero*
          (if (< (length cmd) 3)
              (return-from convert-to-narrow-command nil))
          (case (elt cmd 2)
            (:hand ;; send *aero* :hand
              (if (< (length cmd) 4)
                  (return-from convert-to-narrow-command nil))
              (case (elt cmd 3)
                (:arms ;; send *aero* :hand :arms
                  (if (< (length cmd) 5)
                      (return-from convert-to-narrow-command nil))
                  (case (elt cmd 4)
                    (:joint-angle ;; send *aero* :hand :arms :joint-angle
                      (if (< (length cmd) 6)
                          (return-from convert-to-narrow-command nil))
                      (let ((theta (elt cmd 5)))
                        
                        (ros::publish "/aero_network/command_narrow_band"
                                      (instance aero_network_tools::AeroNarrowCommand :init :theta theta))
                        (send *aero* :hand :arms :joint-angle theta))
                    ) ;; end of case (elt cmd 4) = ::joint-angle
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
  
