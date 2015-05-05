;;
;; test-aero-lower-motion.l
;; 2015- Kotaro Nagahama
;;

(load "aero-lower-interface.l")

;; テスト環境の準備
(defun setup-test-aero-lower-motion
  ()
  (unless (boundp '*aero-lower*)
    (aero-lower-init)
    (objects (list *aero-lower*))
    (setf *aero-lower* *aero-lower*)
    )
  (send *aero-lower* :newcoords (make-coords))
  (reset-terrain-pose)
  )

;; 15/5/2版 適当な姿勢
(defun reset-terrain-pose
  ()
  "#f(0 -56.1797 79.13 0 -56.1797 79.13 0 -56.1797 79.13 0 -56.1797 79.13 0.0)"
  (send *aero-lower* :reset-pose)
  (dolist (link (list :rarm :larm :rleg :lleg))
    (send *aero-lower* link :move-end-pos
	  (float-vector 0 0 200) :local :rotation-axis :y))
  (set-3legs-to-ground)
  (send *irtviewer* :draw-objects)
  )

;; 遊脚を指定して接地，
;; あるいは重さと関係なく，地面に近い３脚を接地させる．
;; end-coordsを地面に合わせるので，実機と誤差は生じるはず
;; （作成中）
(defun set-3legs-to-ground
  (&key (idling-leg nil))
  ;; 単に一脚を接地させるだけのテスト版
  (send *aero-lower* :translate
	(float-vector
	 0 0 (- (aref (send (send *aero-lower* :rarm :end-coords) :worldpos) 2))
	 ) :world)  
  )

;; 四脚の末端位置を固定して，腰を動かして重心を移動する
;; [ex] (move-waist-with-4legs (float-vector 100 0 0)) ;; 100mm前に腰を移動
(defun move-waist-with-4legs
  (&optional (waist-velocity (float-vector 0 0 0)))
  (let ((original-end-coords-list) original-angle-vector
	original-coords)
    ;; 初期状態の保存
    (setf original-angle-vector (send *aero-lower* :angle-vector))
    (setf original-coords (send *aero-lower* :copy-worldcoords))
    (dolist (link (list :rarm :larm :rleg :lleg))
      (push (send *aero-lower* link :end-coords :copy-worldcoords)
	    original-end-coords-list))
    (setf original-end-coords-list (nreverse original-end-coords-list))

    ;; 腰を先に移動
    (send *aero-lower* :translate waist-velocity)

    ;; 四脚を適切な姿勢へ変更
    (dolist (link (list :rarm :larm :rleg :lleg))
      (unless
	  (send *aero-lower* link :inverse-kinematics
		(car original-end-coords-list) :rotation-axis nil)
	;; IKが解けなかった場合は元の状態へ戻す
	(warn ";; move-waist-with-4legs IK failed >> ~S" link)
	(send *aero-lower* :angle-vector original-angle-vector)
	(send *aero-lower* :newcoords original-coords)
	(send *irtviewer* :draw-objects)
	(return nil)
	)
      (setf original-end-coords-list (cdr original-end-coords-list))
      )
    ;;
    (send *irtviewer* :draw-objects)
    ))

;; 重心移動のテスト関数
(defun test-move-waist-with-4legs
  (&optional (vel 2))
  (reset-terrain-pose)
  ;; 前へ重心移動
  (dotimes (i (/ 250 vel))
    (move-waist-with-4legs (float-vector vel 0 0)))
  (dotimes (i (/ 250 vel))
    (move-waist-with-4legs (float-vector (- vel) 0 0)))
  ;; 斜め前へ重心移動
  (dotimes (i (/ 150 vel))
    (move-waist-with-4legs (float-vector vel vel 0)))
  (dotimes (i (/ 150 vel))
    (move-waist-with-4legs (float-vector (- vel) (- vel) 0)))
  ;; 上下へ重心移動
  (dotimes (i (/ 150 vel))
    (move-waist-with-4legs (float-vector 0 0 vel)))
  (dotimes (i (/ 300 vel))
    (move-waist-with-4legs (float-vector 0 0 (- vel))))
  (dotimes (i (/ 150 vel))
    (move-waist-with-4legs (float-vector 0 0 vel)))
  )

(setup-test-aero-lower-motion)

;; TEST
(print '(test-move-waist-with-4legs))
