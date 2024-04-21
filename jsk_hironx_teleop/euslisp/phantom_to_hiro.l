#!/usr/bin/env roseus

(ros::load-ros-manifest "omni_msgs")

(defvar *tf-listener* nil)

(defclass single-arm-handler
  :slots (robot-id arm-name master-dev-name pos-scale force-scale send-rot side master-initial-q hironx-initial-q q-m-q *tf-listener* current-frame-id moving target-pose))

(defmethod single-arm-handler
  (:init
   (&key (robot-id "dual_hironx") (arm-name "larm") (master-dev-name "left-device") (pos-scale 1.0) (force-scale 0.1) (send-rot t) (side "left"))
   (send self :set-val 'master-initial-q (quaternion2matrix -0.5 -0.5 -0.5 0.5))
   (send self :set-val 'hironx-initial-q (quaternion2matrix 1.0 0, 0, 0))
   (send self :set-val 'q-m-p (m* hironx_initial_q (inverse-matrix master-initial-q)))
   (send self :set-val 'current-frame-id (format nil "~A_end_coords" arm-name))
   (send self :set-val 'moving nil)
   (send self :set-val '*tf-listener* (instance ros::transform-listener :init))
  (:setup-ros
   (setq dev-topic (format nil "/~A/phantom/state" master-dev-name))
   (setq pose-target-pub-topic (format nil "~A/~A_target_pose" ros::get-name arm-name))
   (send *tf-listener* :wait-for-transform (format nil "~A_JOINT0_Link" (string-upcase arm-name)) current-frame-id (ros::time-now) 3.6)
   (setq device-state-sub (ros::subscribe dev-topic omni_msgs::OmniState #'send self :device-state-sub))
   (send self :set-val 'target-pose (#'send self :set-initial-pose))

   (setq single-target-pose-pub (ros::advertise pose-target-pub-topic geometry_msgs::PoseStamped 1)))
  (:device-state-sub (msg)
                     ;; not move target pose when locked
                     (if (send msg :locked)
                         (progn
                           (setq moving nil)
                           (break)))
                     ;; update target-pose
                     (setq moving t)
                     (setf (send (send self :get-val 'target-pose) :pose :position :x)
                           (+ (send (send self :get-val 'target-pose) :pose :position :x) (* (- (send msg :pose :position :y) (send target-pose :pose :position :x) pose-scale))))
                     (setf (send (send self :get-val 'target-pose) :pose :position :y) (+ (send target-pose :pose :position :y) (* (- (- (send msg :pose :position :x) (send target-pose :pose :position :y))) pose-scale)))
                     (setf (send (send self :get-val 'target-pose) :pose :position :z) (+ (send target-pose :pose :position :z) (* (- (send msg :pose :position :z) (send target-pose :pose :position :z)) pose-scale)))
                     (if send_rot
                         (progn
                           (setq cur-q (quaternion2matrix (send msg :pose :orientation :x)
                                                          (send msg :pose :orientation :y)
                                                          (send msg :pose :orientation :z)
                                                          (send msg :pose :orientation :x)))
                           (setq tar-q (m* (send self :get-val 'q-m-p) cur-q))
                           (setf (send (send self :get-val 'target-pose) :pose :orientation :x) (+ (send target-pose :pose :orientation :x) (- (elt (matrix2quaternion tar-q) 0))))
                           (setf (send (send self :get-val 'target-pose) :pose :orientation :y) (+ (send target-pose :pose :orientation :y) (elt (matrix2quaternion tar-q) 1)))
                           (setf (send (send self :get-val 'target-pose) :pose :orientation :z) (+ (send target-pose :pose :orientation :z) (- (elt (matrix2quaternion tar-q) 2))))
                           (setf (send (send self :get-val 'target-pose) :pose :orientation :w) (+ (send target-pose :pose :orientation :w) (elt (matrix2quaternion tar-q) 3)))))
                     (setf (send target-pose :header :stamp) (ros::time-now))
                     (ros::publish pose-target-pub-topic target-pose))
  (:set-initial-pose
   (setq current-frame (instance geometry_msgs::PoseStamped :init))
   (setq joint0-link-to-end-effector-transform (send *tfl* :lookup-transform (format nil "~A_JOINT0_Link" (string-upcase arm-name)) current-frame-id (ros::time 0)))
   (setq (send current-frame :pose :position :x) (elt (send (send joint0-link-to-end-effector-transform :copy-worldcoords) :pos) 0))
   (setq (send current-frame :pose :position :y) (elt (send (send joint0-link-to-end-effector-transform :copy-worldcoords) :pos) 1))
   (setq (send current-frame :pose :position :z) (elt (send (send joint0-link-to-end-effector-transform :copy-worldcoords) :pos) 2))
   (setq (send current-frame :pose :orientation :x) (elt (send (send joint0-link-to-end-effector-transform :copy-worldcoords) :rot) 0))
   (setq (send current-frame :pose :orientation :y) (elt (send (send joint0-link-to-end-effector-transform :copy-worldcoords) :rot) 1))
   (setq (send current-frame :pose :orientation :z) (elt (send (send joint0-link-to-end-effector-transform :copy-worldcoords) :rot) 2))
   (setq (send current-frame :pose :orientation :w) (elt (send (send joint0-link-to-end-effector-transform :coyp-worldcoords) :rot) 3))
   (setq (send current-frame :header :stamp) (ros::time-now))
   (setq (send current-frame :header :frame_id) (format nil "~A_JOINT0_Link" (string-upcase arm-name)))
   (return-from current-frame)))


(defclass dual-arm-handler
  (:slots (pos-scale larm-handler)))

(defmethod dual-arm-handler
  (:init
   (&key (pos-scale 1.0))
   (setq larm-handler (instance single-arm-handler :init :arm-name "larm" :master-dev-name "left-device" :pos-scale pos_scale)))
  (:set-initial-pose
   (send larm-handler :set-val 'target-pose (send larm-handler :set-initial-pose)))

  (:loop-call
   (send larm-handler :setup-ros))

  (:run
   (ros::ros-info "Start looping")
   (ros::rate 50)
   (while t
     (ros::sleep))))


(defun main (&rest args)
     (setq node (instance dual-arm-handler :init))
     (send node :run))

(main)
