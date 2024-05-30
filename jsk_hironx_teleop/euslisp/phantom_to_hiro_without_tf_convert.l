#!/usr/bin/env roseus

(load "models/arrow-object.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(load "package://jsk_hironx_teleop/euslisp/hiro-utils.l")

;; (ros::load-ros-manifest "omni_msgs")

(ros::roseus-add-msgs "omni_msgs")
(ros::roseus-add-msgs "jsk_hironx_teleop")

(setq time-a nil)
(setq time-b nil)
(setq interval-time nil)
(setq collection-start nil)

(defun ros-vector3->float-vector (fvec)
  (float-vector (send fvec :x) (send fvec :y) (send fvec :z)))
(defun float-vector->ros-vector3 (fvec)
  (instance geometry_msgs::Vector3 :init
            :x (/ (aref fvec 0) 1000.0)
            :y (/ (aref fvec 1) 1000.0)
            :z (/ (aref fvec 2) 1000.0)))

(defvar *tf-listener* nil)

(defclass single-arm-handler
  :slots (robot-id arm-name master-dev-name pos-scale force-scale send-rot side master-initial-q hironx-initial-q q-m-p target-pose target-force device-target-pos zero-force force-topic force-pub-topic close-gripper device-to-endcoords-transform white-button transformation-matrix robot-state-pos-topic-name robot-state-rot-topic-name robot-state-pos robot-state-rot))

(defmethod single-arm-handler
  (:init
   (&key ((:robot-id _robot-id) "dual_hironx") ((:arm-name _arm-name) "larm") ((:master-dev-name _master-dev-name) "left_device") ((:pos-scale _pos-scale) 1.0) ((:force-scale _force-scale) 0.1) ((:send-rot _send-rot) t) ((:side _side) "left") ((:close-gripper _close-gripper) nil))
   (send self :set-val 'robot-id _robot-id)
   (send self :set-val 'arm-name _arm-name)
   (send self :set-val 'master-dev-name _master-dev-name)
   (send self :set-val 'pos-scale _pos-scale)
   (send self :set-val 'force-scale _force-scale)
   (send self :set-val 'target-force (instance omni_msgs::OmniFeedback :init))
   (send self :set-val 'zero-force nil)
   (send self :set-val 'master-initial-q (quaternion2matrix #f(0.5 -0.5 -0.5 -0.5)))
   (send self :set-val 'hironx-initial-q (quaternion2matrix #f(0 1.0 0 0)))
   (send self :set-val 'transformation-matrix (make-matrix 3 3 (list (list 0 1 0) (list -1 0 0) (list 0 0 1))))
   (send self :set-val 'q-m-p (m* (send self :get-val 'hironx-initial-q) (inverse-matrix (send self :get-val 'master-initial-q))))
   (send self :set-val 'force-pub-topic (format nil "/~A/phantom/force_feedback" (send self :get-val 'master-dev-name)))
   (send self :set-val 'close-gripper _close-gripper)
   (send self :set-val 'white-button "initial")
   (send self :set-val 'robot-state-pos-topic-name (format nil "/jsk_hironx_teleop/~A_arm_state/pos" (if (string= (send self :get-val 'arm-name) "larm") "l" "r")))
   (send self :set-val 'robot-state-rot-topic-name (format nil "/jsk_hironx_teleop/~A_arm_state/rot" (if (string= (send self :get-val 'arm-name) "larm") "l" "r")))
   (send self :set-val 'robot-state-pos nil)
   (send self :set-val 'robot-state-rot nil)
   (setq time-a (ros::time-now))
   (send self :setup-ros))

  (:setup-ros ()
              (setq dev-topic (format nil "/~A/phantom/state" (send self :get-val 'master-dev-name)))
              (send self :set-val 'force-topic (format nil "/~Ahsensor" (if (string= (send self :get-val 'arm-name) "larm") "l" "r")))
              ;; (setq force-pub-topic (format nil "/~A/phantom/force_feedback" (send self :get-val 'master-dev-name)))
              (ros::subscribe dev-topic omni_msgs::OmniState #'send self :device-state-sub 1)
              (ros::subscribe force-topic geometry_msgs::WrenchStamped #'send self :force-cb 1)
              (ros::advertise (send self :get-val 'force-pub-topic) omni_msgs::OmniFeedback 1)
              ;; imitation learing
              (ros::advertise (send self :get-val 'robot-state-pos-topic-name) jsk_hironx_teleop::FloatVector 1)
              (ros::advertise (send self :get-val 'robot-state-rot-topic-name) jsk_hironx_teleop::FloatVector 1)
              (send self :set-val 'target-pose (send self :set-initial-pose)))

  (:device-state-sub (msg)
                     ;; initial device-target-pos
                     (if (not (send self :get-val 'device-target-pos))
                         (send self :set-val 'device-target-pos (send msg :pose :position)))
                     ;; match the value of white button and msg :locked
                     (if (string= white-button "initial")
                         (cond ((send msg :locked) (send self :set-val 'white-button t))
                               ((not (send msg :locked)) (send self :set-val 'white-button nil))))
                     ;; set device-to-endcoords-transform
                     (if (not (send self :get-val 'device-to-endcoords-transform))
                         (progn
                           (setq initial-device-rotation-matrix
                                 (quaternion2matrix
                                  (float-vector (send msg :pose :orientation :w)
                                                (send msg :pose :orientation :x)
                                                (send msg :pose :orientation :y)
                                                (send msg :pose :orientation :z))))
                           (setq adjust-initial-device-rotation-matrix
                                 (m* (send self :get-val 'transformation-matrix)
                                     (m* initial-device-rotation-matrix
                                         (transpose (send self :get-val 'transformation-matrix)))))
                           (setq initial-device-coords
                                 (send (make-coords) :Euler
                                       (elt (elt (euler-angle adjust-initial-device-rotation-matrix) 0) 0)
                                       (elt (elt (euler-angle adjust-initial-device-rotation-matrix) 0) 1)
                                       (elt (elt (euler-angle adjust-initial-device-rotation-matrix) 0) 2)))
                           (send self :set-val 'device-to-endcoords-transform
                                 (if (string= (send self :get-val 'arm-name) "larm")
                                     (send (send initial-device-coords :copy-worldcoords) :transformation (send *hironxjsk* :larm :end-coords :copy-worldcoords))
                                   (send (send initial-device-coords :copy-worldcoords) :transformation (send *hironxjsk* :rarm :end-coords :copy-worldcoords))))))
                     ;; gripper's states
                     (if (and (not close-gripper) (send msg :close_gripper))
                         (progn
                           (send self :set-val 'close-gripper t)
                           (if (string= (send self :get-val 'arm-name) "larm")
                               (send *hironxjsk* :start-grasp :larm)
                             (send *hironxjsk* :start-grasp :rarm))
                           (ir2ri-hand)))
                     (if (and close-gripper (not (send msg :close_gripper)))
                         (progn
                           (send self :set-val 'close-gripper nil)
                           (if (string= (send self :get-val 'arm-name) "larm")
                               (send *hironxjsk* :stop-grasp :larm)
                             (send *hironxjsk* :stop-grasp :rarm))
                           (ir2ri-hand)))
                     ;; Process when changing from unlocked to locked
                     (if (and (not (send self :get-val 'white-button)) (send msg :locked))
                         (progn
                           (send self :set-val 'white-button t)
                           (send self :cal-interval-time)
                           (cond ((and (not collection-start) (< interval-time 1))
                                  (progn
                                    (send self :data-collection-start)
                                    (setq collection-start t)))
                                 ((and collection-start (< interval-time 1))
                                  (send self :data-collection-stop)))))
                     ;; Process when changing from locked to unlocked
                     (if (and (send self :get-val 'white-button) (not (send msg :locked)))
                         (progn
                           (send self :cal-interval-time)
                           (send self :set-val 'device-target-pos (send msg :pose :position))
                           (send self :set-val 'white-button nil)
                           (cond ((and (not collection-start) (< interval-time 1))
                                  (progn
                                    (send self :data-collection-start)
                                    (setq collection-start t)))
                                 ((and collection-start (< interval-time 1))
                                  (send self :data-collection-stop)))))
                     ;; when unlocked
                     (if (not (send self :get-val 'white-button))
                         (progn
                           ;; update target-pose
                           (send (send self :get-val 'target-pose) :translate
                                 (float-vector (* (- (send msg :pose :position :y) (send (send self :get-val 'device-target-pos) :y)) (send self :get-val 'pos-scale))
                                               (* (- (- (send msg :pose :position :x) (send (send self :get-val 'device-target-pos) :x))) (send self :get-val 'pos-scale))
                                               (* (- (send msg :pose :position :z) (send (send self :get-val 'device-target-pos) :z)) (send self :get-val 'pos-scale))) :local)
                           (send self :set-val 'device-target-pos (send msg :pose :position))
                           (if send-rot
                               (ros::ros-info "test")
                               (progn
                                 (setq cur-device-q
                                       (quaternion2matrix (float-vector
                                                           (send msg :pose :orientation :w)
                                                           (send msg :pose :orientation :x)
                                                           (send msg :pose :orientation :y)
                                                           (send msg :pose :orientation :z))))
                                 (setq adjust-cur-device-q
                                       (m* (send self :get-val 'transformation-matrix)
                                           (m* cur-device-q (transpose (send self :get-val 'transformation-matrix)))))
                                 (setq cur-device-rot
                                       (send (make-coords) :Euler
                                             (elt (elt (euler-angle adjust-cur-device-q) 0) 0)
                                             (elt (elt (euler-angle adjust-cur-device-q) 0) 1)
                                             (elt (elt (euler-angle adjust-cur-device-q) 0) 2)))
                                 (send cur-device-rot :transform (send self :get-val 'device-to-endcoords-transform))
                                 (send (send self :get-val 'target-pose) :Euler
                                       (elt (elt (euler-angle (send cur-device-rot :rot)) 0) 0)
                                       (elt (elt (euler-angle (send cur-device-rot :rot)) 0) 1)
                                       (elt (elt (euler-angle (send cur-device-rot :rot)) 0) 2)))))))

  (:force-cb (msg)
             (if (eq (send self :get-val 'zero-force) nil)
                 (send self :set-val 'zero-force (float-vector (send msg :wrench :force :x)
                                                               (send msg :wrench :force :y)
                                                               (send msg :wrench :force :z))))
             (setq float-vector-target-force (ros-vector3->float-vector (send (send self :get-val 'target-force) :force)))
             (setf (elt float-vector-target-force 0) (* (- (send msg :wrench :force :y) (elt (send self :get-val 'zero-force) 0)) (send self :get-val 'force-scale)))
             (setf (elt float-vector-target-force 1) (* (- (- (send msg :wrench :force :x) (elt (send self :get-val 'zero-force) 1))) (send self :get-val 'force-scale)))
             (setf (elt float-vector-target-force 2) (* (- (send msg :wrench :force :z) (elt (send self :get-val 'zero-force) 2)) (send self :get-val 'force-scale)))
             (send (send self :get-val 'target-force) :force (float-vector->ros-vector3 float-vector-target-force)))

  (:apply-target-force ()
                       (ros::publish (send self :get-val 'force-pub-topic) (send self :get-val 'target-force)))

  (:update-robot-state ()
                       (let (end-effector-pos end-effector-rot)
                         (if (string= (send self :get-val 'arm-name) "larm")
                             (progn
                               (setq end-effector-pos (send (send *hironxjsk* :larm :end-coords :copy-worldcoords) :worldpos))
                               (setq end-effector-rot (matrix2quaternion (send (send *hironxjsk* :larm :end-coords :copy-worldcoords) :worldrot))))
                           (progn
                             (setq end-effector-pos (send (send *hironxjsk* :rarm :end-coords :copy-worldcoords) :worldpos))
                             (setq end-effector-rot (matrix2quaternion (send (send *hironxjsk* :rarm :end-coords :copy-worldcoords) :worldrot)))))
                         (send self :set-val 'robot-state-pos end-effector-pos)
                         (send self :set-val 'robot-state-rot end-effector-rot)))

  (:publish-robot-state ()
                        (let ((robot-state-pos-msg (instance jsk_hironx_teleop::FloatVector :init))
                              (robot-state-rot-msg (instance jsk_hironx_teleop::FloatVector :init)))
                          (send robot-state-pos-msg :data (send self :get-val 'robot-state-pos))
                          (send robot-state-pos-msg :header :stamp (ros::time-now))
                          (send robot-state-rot-msg :data (send self :get-val 'robot-state-rot))
                          (send robot-state-rot-msg :header :stamp (ros::time-now))
                          (ros::publish robot-state-pos-topic-name robot-state-pos-msg)
                          (ros::publish robot-state-rot-topic-name robot-state-rot-msg)
                          ))

  (:set-initial-pose ()
                     (if (string= (send self :get-val 'arm-name) "larm")
                         (setq initial-target-pose (send *hironxjsk* :larm :end-coords :copy-worldcoords))
                       (setq initial-target-pose (send *hironxjsk* :rarm :end-coords :copy-worldcoords))))

  (:adjust-rotation-matrix (rotation-matrix)
                           (setq tar-q (m* (send self :get-val 'q-m-p) rotation-matrix))
                           (setq adjust-quaternion
                                 (float-vector (elt (matrix2quaternion tar-q) 0)
                                               (elt (matrix2quaternion tar-q) 1)
                                               (elt (matrix2quaternion tar-q) 2)
                                               (elt (matrix2quaternion tar-q) 3)))
                           (setq adjust-rotation-matrix (quaternion2matrix adjust-quaternion)))
  (:cal-interval-time ()
                      (setq time-b (ros::time-now))
                      (setq interval-time (- (send time-b :sec) (send time-a :sec)))
                      (setq time-a time-b))

  (:data-collection-start ()
                          (ros::publish "/jsk_hironx_teleop/data_collector/record" (instance std_msgs::String :init :data "start")))

  (:data-collection-stop ()
                        (ros::publish "/jsk_hironx_teleop/data_collector/record" (instance std_msgs::String :init :data "stop")))

  (:run ()
        (ros::spin-once)))

(defclass dual-arm-handler
  :slots (pos-scale-dual force-scale-dual larm-handler))

(defmethod dual-arm-handler
  (:init
   (&key ((:pos-scale-dual _pos-scale-dual) 1.0) ((:force-scale-dual _force-scale-dual) 0.1))
   (ros::roseus "dual-phantom-master")
   (setq *left-target-arrow* (arrow))
   (setq *left-end-coords* (arrow))
   (send *left-end-coords* :newcoords (send *hironxjsk* :larm :end-coords :copy-worldcoords))
   (send (send *hironxjsk* :larm :end-coords) :assoc *left-end-coords*)
   (objects (list *hironxjsk* *left-target-arrow* *left-end-coords*))
   (send self :set-val 'pos-scale-dual _pos-scale-dual)
   (send self :set-val 'force-scale-dual _force-scale-dual)
   (send self :set-val 'larm-handler (instance single-arm-handler :init :arm-name "larm" :master-dev-name "left_device" :pos-scale (send self :get-val 'pos-scale-dual) :force-scale (send self :get-val 'force-scale-dual)))
   (send (send self :get-val 'larm-handler) :run)
   (send *left-target-arrow* :move-to (send *hironxjsk* :larm :end-coords :copy-worldcoords) :world)
   (ros::advertise "/jsk_hironx_teleop/data_collector/record" std_msgs::String 1))

  (:loop-call ()
              (send (send self :get-val 'larm-handler) :run)
              (send *left-target-arrow* :move-to (send (send (send self :get-val 'larm-handler) :get-val 'target-pose) :copy-worldcoords) :world)
              (solve-ik-larm (send *left-target-arrow* :copy-worldcoords))
              (send *ri* :angle-vector (send *hironxjsk* :angle-vector) (* 1.5 (/ 1000 50)))
              ;; (send *ri* :wait-interpolation)
              (send *irtviewer* :draw-objects)
              ;; force
              (send (send self :get-val 'larm-handler) :apply-target-force)
              ;; imitation
              ;; (send (send self :get-val 'larm-handler) :update-robot-state)
              ;; (send (send self :get-val 'larm-handler) :publish-robot-state)
              )

  (:run ()
   (ros::ros-info "Start looping")
   (ros::rate 50)
   (while t
     (send self :loop-call))))


(defun main (&rest args)
  (setq node (instance dual-arm-handler :init :pos-scale-dual 1.0 :force-scale-dual 50.0))
  (send node :run))

(main)
