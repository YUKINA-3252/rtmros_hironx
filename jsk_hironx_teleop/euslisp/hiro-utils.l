#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")

(hironxjsk-init)

;; connection with robot and irt
(defun ir2ri (&optional (time 3000))
  (send *ri* :angle-vector (send *hironxjsk* :angle-vector) time)
  (send *ri* :hand-angle-vector :rhand (send *hironxjsk* :hand-angle-vector :rhand))
  (send *ri* :hand-angle-vector :lhand (send *hironxjsk* :hand-angle-vector :lhand))
  (send *ri* :wait-interpolation))
(defun ir2ri-hand ()
    (send *ri* :hand-angle-vector :rhand (send *hironxjsk* :hand-angle-vector :rhand))
    (send *ri* :hand-angle-vector :lhand (send *hironxjsk* :hand-angle-vector :lhand))
    (send *ri* :wait-interpolation))
(defun ri2ir ()
  (send *hironxjsk* :angle-vector (send *ri* :state :potentio-vector)))

;; ik
(defun solve-ik-larm (target-coords)
  (send *hironxjsk* :larm :inverse-kinematics (send target-coords :copy-worldcoords) :rotation-axis t))
(defun solve-ik-rarm (target-coords)
  (send *hironxjsk* :rarm :inverse-kinematics (send target-coords :copy-worldcoords) :rotation-axis t))
