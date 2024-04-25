#!/usr/bin/env roseus

(load "models/arrow-object.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
;; (ros::load-ros-manifest "omni_msgs")

(ros::roseus-add-msgs "omni_msgs")

(defun solve-ik-larm (target-coords)
  (send *hironxjsk* :larm :inverse-kinematics (send target-coords :copy-worldcoords) :rotation-axis t))
(defun solve-ik-rarm (target-coords)
  (send *hironxjsk* :rarm :inverse-kinematics (send target-coords :copy-worldcoords) :rotation-axis t))

(hironxjsk-init)

(defvar *tf-listener* nil)

(defclass single-arm-handler
  :slots (robot-id arm-name master-dev-name pos-scale force-scale send-rot side master-initial-q hironx-initial-q q-m-p *tf-listener* current-frame-id moving target-pose device-target-pos))

(defmethod single-arm-handler
  (:init
   (&key ((:robot-id _robot-id) "dual_hironx") ((:arm-name _arm-name) "larm") ((:master-dev-name _master-dev-name) "left_device") ((:pos-scale _pos-scale) 1.0) ((:force-scale _force-scale) 0.1) ((:send-rot _send-rot) t) ((:side _side) "left"))
   (send self :set-val 'robot-id _robot-id)
   (send self :set-val 'arm-name _arm-name)
   (send self :set-val 'master-dev-name _master-dev-name)
   (send self :set-val 'pos-scale _pos-scale)
   (send self :set-val 'force-scale _force-scale)
   (send self :set-val 'master-initial-q (quaternion2matrix #f(0.5 -0.5 -0.5 -0.5)))
   (send self :set-val 'hironx-initial-q (quaternion2matrix #f(0 1.0 0 0)))
   (send self :set-val 'q-m-p (m* (send self :get-val 'hironx-initial-q) (inverse-matrix (send self :get-val 'master-initial-q))))
   (send self :set-val 'current-frame-id (format nil "~A_end_coords" (send self :get-val 'arm-name)))
   (send self :set-val 'moving nil)
   (send self :set-val '*tf-listener* (instance ros::transform-listener :init))
   (send self :setup-ros)
   )
  (:setup-ros ()
              ;; (setq pose-target-pub-topic (format nil "phantom_to_hironx/~A_target_pose" (send self :get-val 'arm-name)))
              (send *tf-listener* :wait-for-transform (format nil "~A_JOINT0_Link" (string-upcase (send self :get-val 'arm-name))) (send self :get-val 'current-frame-id) (ros::time-now) 3.6)
              (send self :set-val 'target-pose (send self :set-initial-pose))
              ;; (setq single-target-pose-pub (ros::advertise pose-target-pub-topic geometry_msgs::PoseStamped 1))
              )

  (:device-state-sub (msg)
                     ;; not move target pose when locked
                     (if (send msg :locked)
                         (progn
                           (setq moving nil)
                           (break)))
                     ;; initial device-target-pos
                     (if (not (send self :get-val 'device-target-pos))
                         (send self :set-val 'device-target-pos (send msg :pose :position)))
                     ;; update target-pose
                     (setq moving t)
                     (setf (elt (send (send self :get-val 'target-pose) :pos) 0)
                           (+ (elt (send (send self :get-val 'target-pose) :pos) 0) (* (- (- (send msg :pose :position :y) (send (send self :get-val 'device-target-pos) :y))) (send self :get-val 'pos-scale))))
                     (setf (elt (send (send self :get-val 'target-pose) :pos) 1)
                           (+ (elt (send (send self :get-val 'target-pose) :pos) 1) (* (- (- (send msg :pose :position :x) (send (send self :get-val 'device-target-pos) :x)) (send self :get-val 'pos-scale)))))
                     (setf (elt (send (send self :get-val 'target-pose) :pos) 2)
                           (+ (elt (send (send self :get-val 'target-pose) :pos) 2) (* (- (send msg :pose :position :z) (send (send self :get-val 'device-target-pos) :z)) (send self :get-val 'pos-scale))))
                     (send self :set-val 'device-target-pos (send msg :pose :position))
                     (if send-rot
                         (progn
                           (setq cur-q (quaternion2matrix (float-vector
                                                           (send msg :pose :orientation :w)
                                                           (send msg :pose :orientation :x)
                                                           (send msg :pose :orientation :y)
                                                           (send msg :pose :orientation :z))))
                           (setq tar-q (m* (send self :get-val 'q-m-p) cur-q))
                           (setq adjust-tar-q (quaternion2matrix (float-vector
                                                                  (- (elt (matrix2quaternion tar-q) 0))
                                                                  (elt (matrix2quaternion tar-q) 1)
                                                                  (- (elt (matrix2quaternion tar-q) 2))
                                                                  (elt (matrix2quaternion tar-q) 3))))
                           (send (send self :get-val 'target-pose) :Euler
                                 (elt (elt (euler-angle adjust-tar-q) 0) 0)
                                 (elt (elt (euler-angle adjust-tar-q) 0) 1)
                                 (elt (elt (euler-angle adjust-tar-q) 0) 2))
                           ))
                     )
  (:set-initial-pose ()
                     (setq current-frame (make-coords))
                     (setq joint0-link-to-end-effector-transform (send *tfl* :lookup-transform (format nil "~A_JOINT0_Link" (string-upcase (send self :get-val 'arm-name))) current-frame-id (ros::time 0)))
                     (send current-frame :move-to (send joint0-link-to-end-effector-transform :copy-worldcoords)))

  (:transform-waist-based ()
                          (setq base-to-joint0-link (send *tfl* :lookup-transform
                                                          "WAIST" (format nil "~A_JOINT0_Link" (string-upcase (send self :get-val 'arm-name))) (ros::time 0)))
                          (setq base-to-target-transform (send (send base-to-joint0-link :copy-worldcoords) :transform (send (send self :get-val 'target-pose) :copy-worldcoords))))

  (:run ()
        (setq dev-topic (format nil "/~A/phantom/state" (send self :get-val 'master-dev-name)))
        (ros::subscribe dev-topic omni_msgs::OmniState #'send self :device-state-sub 2000)
        (ros::spin-once)
        )
  )

(defclass dual-arm-handler
  :slots (pos-scale-dual larm-handler device-to-endcoords-transform))

(defmethod dual-arm-handler
  (:init
   (&key ((:pos-scale-dual _pos-scale-dual) 1.0))
   (ros::roseus "dual-phantom-master")
   (setq *left-target-arrow* (arrow))
   (setq *left-end-coords* (arrow))
   (send *left-end-coords* :newcoords (send *hironxjsk* :larm :end-coords :copy-worldcoords))
   (send (send *hironxjsk* :larm :end-coords) :assoc *left-end-coords*)
   (objects (list *hironxjsk* *left-target-arrow* *left-end-coords*))
   (send self :set-val 'pos-scale-dual _pos-scale-dual)
   (send self :set-val 'larm-handler (instance single-arm-handler :init :arm-name "larm" :master-dev-name "left_device" :pos-scale (send self :get-val 'pos-scale-dual)))
   (send (send self :get-val 'larm-handler) :run)
   (send *left-target-arrow* :move-to (send (send (send self :get-val 'larm-handler) :transform-waist-based) :copy-worldcoords) :world)
   (send self :set-val 'device-to-endcoords-transform (send (send *left-target-arrow* :copy-worldcoords) :transformation (send *left-end-coords* :copy-worldcoords)))
)

  (:loop-call ()
              (send (send self :get-val 'larm-handler) :run)
              (send *left-target-arrow* :move-to (send (send (send (send self :get-val 'larm-handler) :transform-waist-based) :copy-worldcoords) :transform (send (send self :get-val 'device-to-endcoords-transform) :copy-worldcoords)) :world)
              (solve-ik-larm (send *left-target-arrow* :copy-worldcoords))
              (send *ri* :angle-vector (send *hironxjsk* :angle-vector) 1000)
              (send *irtviewer* :draw-objects)
              (unix::sleep 1))

  (:run ()
   (ros::ros-info "Start looping")
   (ros::rate 50)
   (while t
     (send self :loop-call)
     (ros::sleep)))
  )


(defun main (&rest args)
  (setq node (instance dual-arm-handler :init))
  (send node :run))

(main)
