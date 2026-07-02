#!/usr/bin/env roseus

(require "package://dynamixel_detachable_hand/euslisp/rhand-interface.l")
(require "package://dynamixel_detachable_hand/euslisp/lhand-interface.l")

(defvar *detachable-tool-robot* nil)
(defvar *detachable-tool-robot-kind* nil)
(defvar *detachable-tool-arm-map* nil)
(defvar *detachable-tool-state* nil)
(defvar *detachable-tool-launch-wait* 3.0)
(defparameter *detachable-tool-needle-holder-grasp-tip-ops*
  (list (list :rotate (deg2rad -90) :y)
        (list :rotate (deg2rad 90) :x :local)
        (list :translate #f(96 -12 -20))))

(defun detachable-tool-key-string (x)
  (string-upcase (string x)))

(defun detachable-tool-normalize-arm (arm)
  (let ((s (detachable-tool-key-string arm)))
    (cond
     ((string= s "RARM") :rarm)
     ((string= s "LARM") :larm)
     (t (error "unsupported arm ~A" arm)))))

(defun detachable-tool-normalize-side (side)
  (let ((s (detachable-tool-key-string side)))
    (cond
     ((string= s "RHAND") :rhand)
     ((string= s "LHAND") :lhand)
     (t (error "unsupported detachable hand side ~A" side)))))

(defun detachable-tool-normalize-tool (tool)
  (let ((s (detachable-tool-key-string tool)))
    (cond
     ((or (string= s "GENERIC") (string= s "DUMMY")) :generic)
     ((or (string= s "NEEDLE_HOLDER")
          (string= s "NEEDLE-HOLDER")
          (string= s "HOLDER")) :needle_holder)
     ((string= s "GRIPPER") :gripper)
     ((or (string= s "FORCEPS")
          (string= s "SCISSORS")
          (string= s "SCISSOR")) :forceps)
     (t (error "unsupported detachable tool ~A" tool)))))

(defun detachable-tool-normalize-robot-kind (kind)
  (let ((s (detachable-tool-key-string kind)))
    (cond
     ((or (string= s "MYCOBOT") (string= s "SINGLE-ARM")
          (string= s "SINGLE_ARM")) :mycobot)
     ((or (string= s "NEXTAGE") (string= s "DUAL-ARM")
          (string= s "DUAL_ARM")) :nextage)
     (t (error "unsupported robot kind ~A" kind)))))

(defun detachable-tool-tool-name (tool)
  (case (detachable-tool-normalize-tool tool)
    (:generic "generic")
    (:needle_holder "needle_holder")
    (:gripper "gripper")
    (:forceps "forceps")))

(defun detachable-tool-side-name (side)
  (case (detachable-tool-normalize-side side)
    (:rhand "rhand")
    (:lhand "lhand")))

(defun detachable-tool-default-arm-map (robot-kind)
  (case (detachable-tool-normalize-robot-kind robot-kind)
    (:mycobot '((:rarm . :lhand)))
    (:nextage '((:larm . :lhand) (:rarm . :rhand)))))

(defun detachable-tool-guess-robot-kind (robot)
  (cond
   ((and (boundp '*mycobot*) (eq robot *mycobot*)) :mycobot)
   ((and (boundp '*nextage*) (eq robot *nextage*)) :nextage)
   (t :mycobot)))

(defun tool-setup-robot (robot &key robot-kind arm-map)
  (setq *detachable-tool-robot* robot)
  (setq *detachable-tool-robot-kind*
        (detachable-tool-normalize-robot-kind
         (or robot-kind (detachable-tool-guess-robot-kind robot))))
  (setq *detachable-tool-arm-map*
        (or arm-map (detachable-tool-default-arm-map *detachable-tool-robot-kind*)))
  *detachable-tool-arm-map*)

(defun detachable-tool-side-for-arm (robot arm)
  (unless *detachable-tool-arm-map*
    (tool-setup-robot robot))
  (let* ((a (detachable-tool-normalize-arm arm))
         (pair (assoc a *detachable-tool-arm-map*)))
    (unless pair
      (error "arm ~A is not available for detachable tools; arm map is ~A"
             a *detachable-tool-arm-map*))
    (detachable-tool-normalize-side (cdr pair))))

(defun detachable-tool-arm-state (arm)
  (cdr (assoc (detachable-tool-normalize-arm arm) *detachable-tool-state*)))

(defun detachable-tool-set-arm-state (arm state)
  (let* ((a (detachable-tool-normalize-arm arm))
         (entry (assoc a *detachable-tool-state*)))
    (if entry
        (rplacd entry state)
      (push (cons a state) *detachable-tool-state*))
    state))

(defun detachable-tool-state-get (arm key)
  (cdr (assoc key (detachable-tool-arm-state arm))))

(defun detachable-tool-state-set (arm key value)
  (let* ((state (detachable-tool-arm-state arm))
         (entry (assoc key state)))
    (if entry
        (rplacd entry value)
      (setq state (acons key value state)))
    (detachable-tool-set-arm-state arm state)
    value))

(defun detachable-tool-clear-arm-state (arm)
  (let ((entry (assoc (detachable-tool-normalize-arm arm)
                      *detachable-tool-state*)))
    (when entry
      (setq *detachable-tool-state*
            (remove entry *detachable-tool-state* :test #'eq)))))

(defun detachable-tool-hand-interface (side)
  (case (detachable-tool-normalize-side side)
    (:rhand (rhand-init))
    (:lhand (lhand-init))))

(defun detachable-tool-shell (cmd)
  (ros::ros-info "~A" cmd)
  (unix:system cmd))

(defun detachable-tool-bool-string (value)
  (if value "true" "false"))

(defun detachable-tool-launch-static-tf-args
    (publish-static-tf parent-frame root-frame static-xyz static-rpy)
  (if (or publish-static-tf parent-frame)
      (format nil " publish_static_tf:=~A parent_frame:=~A root_frame:=~A static_xyz:='~A' static_rpy:='~A'"
              (detachable-tool-bool-string (or publish-static-tf parent-frame))
              (or parent-frame "")
              (or root-frame "")
              (or static-xyz "0 0 0")
              (or static-rpy "0 0 0"))
    ""))

(defun detachable-tool-kill-model-nodes (namespace)
  (detachable-tool-shell
   (format nil "rosnode kill /~A/loop_coupling_relay /~A/robot_state_publisher /~A/tool_mount_static_tf >/dev/null 2>&1"
           namespace namespace namespace))
  (detachable-tool-shell
   (format nil "rosparam delete /~A/robot_description >/dev/null 2>&1"
           namespace)))

(defun reload-tool-model (side tool &key namespace
                                    publish-static-tf parent-frame root-frame
                                    (static-xyz "0 0 0")
                                    (static-rpy "0 0 0")
                                    (wait *detachable-tool-launch-wait*))
  (let* ((side-name (detachable-tool-side-name side))
         (ns (or namespace side-name))
         (tool-name (detachable-tool-tool-name tool))
         (root (or root-frame (format nil "~A_base_link" side-name)))
         (static-tf-args
          (detachable-tool-launch-static-tf-args
           publish-static-tf parent-frame root static-xyz static-rpy)))
    (detachable-tool-kill-model-nodes ns)
    (detachable-tool-shell
     (format nil "roslaunch dynamixel_detachable_hand hand_tool_model.launch side:=~A namespace:=~A tool:=~A~A &"
             side-name ns tool-name static-tf-args))
    (when wait (ros::duration-sleep wait))
    tool-name))

(defun start-tool-control (side tool &key namespace port-name
                                      (baud-rate "57600")
                                      publish-static-tf parent-frame root-frame
                                      (static-xyz "0 0 0")
                                      (static-rpy "0 0 0")
                                      (wait *detachable-tool-launch-wait*))
  (let* ((side-name (detachable-tool-side-name side))
         (ns (or namespace side-name))
         (tool-name (detachable-tool-tool-name tool))
         (launch-file (case (detachable-tool-normalize-side side)
                        (:rhand "right_hand.launch")
                        (:lhand "left_hand.launch")))
         (port (or port-name (format nil "/dev/~A" side-name)))
         (root (or root-frame (format nil "~A_base_link" side-name)))
         (static-tf-args
          (detachable-tool-launch-static-tf-args
           publish-static-tf parent-frame root static-xyz static-rpy)))
    (detachable-tool-shell
     (format nil "rosnode kill /~A/controller_spawner /~A/dynamixel_general_control /~A/loop_coupling_relay /~A/robot_state_publisher /~A/tool_mount_static_tf >/dev/null 2>&1"
             ns ns ns ns ns))
    (detachable-tool-shell
     (format nil "roslaunch dynamixel_detachable_hand ~A namespace:=~A port_name:=~A baud_rate:=~A tool:=~A~A &"
             launch-file ns port baud-rate tool-name static-tf-args))
    (when wait (ros::duration-sleep wait))
    tool-name))

(defun start-detacher-control (side &key namespace port-name
                                     (baud-rate "57600")
                                     (wait *detachable-tool-launch-wait*))
  (let* ((side-name (detachable-tool-side-name side))
         (ns (or namespace side-name))
         (launch-file (case (detachable-tool-normalize-side side)
                        (:rhand "right_detacher.launch")
                        (:lhand "left_detacher.launch")))
         (port (or port-name (format nil "/dev/~A" side-name))))
    (detachable-tool-shell
     (format nil "rosnode kill /~A/controller_spawner /~A/dynamixel_general_control /~A/loop_coupling_relay /~A/robot_state_publisher >/dev/null 2>&1"
             ns ns ns ns))
    (detachable-tool-shell
     (format nil "roslaunch dynamixel_detachable_hand ~A namespace:=~A port_name:=~A baud_rate:=~A &"
             launch-file ns port baud-rate))
    (when wait (ros::duration-sleep wait))
    ns))

(defun detachable-tool-default-tip-ops (robot-kind arm tool)
  (let ((kind (detachable-tool-normalize-robot-kind robot-kind))
        (a (detachable-tool-normalize-arm arm))
        (tname (detachable-tool-normalize-tool tool)))
    (cond
     ((and (eq kind :nextage) (eq a :rarm) (eq tname :needle_holder))
      (list (list :translate #f(0 0 190))
            (list :translate #f(0 0 -102))
            (list :rotate (deg2rad 33) :z)
            (list :rotate (deg2rad 88.7) :y)
            (list :translate #f(0 -23 90))))
     ((and (eq kind :nextage) (eq a :larm) (eq tname :forceps))
      (list (list :rotate (deg2rad -90) :y)
            (list :translate #f(190 0 0))
            (list :translate #f(-170 -34 -45))
            (list :rotate (deg2rad -45) :x)
            (list :rotate (deg2rad -20.534) :z :local)
            (list :rotate (deg2rad 24.683) :y :local)
            (list :rotate (deg2rad 138.110) :x :local)
            (list :rotate (deg2rad 180) :z :local)
            (list :translate #f(0 0 5))))
     ((eq tname :needle_holder)
      *detachable-tool-needle-holder-grasp-tip-ops*)
     ((eq tname :forceps)
      (list (list :translate #f(0 0 90))))
     ((eq tname :gripper)
      (list (list :translate #f(0 0 80))))
     (t
      (list (list :translate #f(0 0 60)))))))

(defun detachable-tool-tip-ops-from-pos-rpy (pos rpy)
  (append
   (if pos (list (list :translate pos)) nil)
   (if rpy
       (list (list :rotate (elt rpy 0) :x :local)
             (list :rotate (elt rpy 1) :y :local)
             (list :rotate (elt rpy 2) :z :local))
     nil)))

(defun detachable-tool-apply-tip-op (coords op)
  (case (car op)
    (:translate
     (if (cddr op)
         (send coords :translate (cadr op) (caddr op))
       (send coords :translate (cadr op))))
    (:rotate
     (if (cdddr op)
         (send coords :rotate (cadr op) (caddr op) (cadddr op))
       (send coords :rotate (cadr op) (caddr op))))
    (t (error "unsupported tool tip op ~A" op)))
  coords)

(defun detachable-tool-install-tip (robot arm tool &key tip-ops tip-pos tip-rpy)
  (let* ((a (detachable-tool-normalize-arm arm))
         (tname (detachable-tool-normalize-tool tool))
         (end-coords (send robot a :end-coords))
         (tip (make-cascoords
               :name (format nil "~A-~A-tip" (string-downcase (string a))
                             (detachable-tool-tool-name tname))))
         (ops (or tip-ops
                  (detachable-tool-tip-ops-from-pos-rpy tip-pos tip-rpy)
                  (detachable-tool-default-tip-ops *detachable-tool-robot-kind*
                                                   a tname))))
    (detach-tool-tip robot a)
    (send end-coords :assoc tip)
    (send tip :newcoords (make-coords))
    (dolist (op ops) (detachable-tool-apply-tip-op tip op))
    (detachable-tool-state-set a :tool tname)
    (detachable-tool-state-set a :tip tip)
    (detachable-tool-state-set a :tip-ops ops)
    tip))

(defun detach-tool-tip (robot arm)
  (let* ((a (detachable-tool-normalize-arm arm))
         (tip (detachable-tool-state-get a :tip)))
    (when (and tip (send tip :parent))
      (send (send tip :parent) :dissoc tip))
    (detachable-tool-state-set a :tip nil)
    nil))

(defun attach-tool (robot arm tool &rest args
                          &key (physical nil)
                               (reload-model t)
                               (restart-control nil)
                               namespace port-name
                               (baud-rate "57600")
                               (tm 6.0)
                               (wait *detachable-tool-launch-wait*)
                               tip-ops tip-pos tip-rpy
                               publish-static-tf parent-frame root-frame
                               (static-xyz "0 0 0")
                               (static-rpy "0 0 0")
                          &allow-other-keys)
  (declare (ignore args))
  (unless *detachable-tool-arm-map*
    (tool-setup-robot robot))
  (let* ((a (detachable-tool-normalize-arm arm))
         (side (detachable-tool-side-for-arm robot a))
         (tname (detachable-tool-normalize-tool tool)))
    (when physical
      (send (detachable-tool-hand-interface side) :attach :tm tm))
    (cond
     (restart-control
      (start-tool-control side tname :namespace namespace :port-name port-name
                          :baud-rate baud-rate :wait wait
                          :publish-static-tf publish-static-tf
                          :parent-frame parent-frame
                          :root-frame root-frame
                          :static-xyz static-xyz
                          :static-rpy static-rpy))
     (reload-model
      (reload-tool-model side tname :namespace namespace :wait wait
                         :publish-static-tf publish-static-tf
                         :parent-frame parent-frame
                         :root-frame root-frame
                         :static-xyz static-xyz
                         :static-rpy static-rpy)))
    (detachable-tool-state-set a :side side)
    (detachable-tool-state-set a :publish-static-tf publish-static-tf)
    (detachable-tool-state-set a :parent-frame parent-frame)
    (detachable-tool-state-set a :root-frame root-frame)
    (detachable-tool-state-set a :static-xyz static-xyz)
    (detachable-tool-state-set a :static-rpy static-rpy)
    (detachable-tool-install-tip robot a tname
                                 :tip-ops tip-ops
                                 :tip-pos tip-pos
                                 :tip-rpy tip-rpy)))

(defun detach-tool (robot arm &rest args
                          &key (physical nil)
                               (reload-model t)
                               (restart-control nil)
                               namespace port-name
                               (baud-rate "57600")
                               (tm 6.0)
                               (wait *detachable-tool-launch-wait*)
                               publish-static-tf parent-frame root-frame
                               static-xyz static-rpy
                          &allow-other-keys)
  (declare (ignore args))
  (unless *detachable-tool-arm-map*
    (tool-setup-robot robot))
  (let* ((a (detachable-tool-normalize-arm arm))
         (side (detachable-tool-side-for-arm robot a))
         (static-p (or publish-static-tf
                       (detachable-tool-state-get a :publish-static-tf)
                       parent-frame
                       (detachable-tool-state-get a :parent-frame)))
         (parent (or parent-frame
                     (detachable-tool-state-get a :parent-frame)))
         (root (or root-frame
                   (detachable-tool-state-get a :root-frame)))
         (xyz (or static-xyz
                  (detachable-tool-state-get a :static-xyz)
                  "0 0 0"))
         (rpy (or static-rpy
                  (detachable-tool-state-get a :static-rpy)
                  "0 0 0")))
    (when physical
      (send (detachable-tool-hand-interface side) :detach :tm tm))
    (detach-tool-tip robot a)
    (detachable-tool-state-set a :tool :generic)
    (when restart-control
      (start-detacher-control side :namespace namespace :port-name port-name
                              :baud-rate baud-rate :wait wait))
    (when reload-model
      (reload-tool-model side :generic :namespace namespace :wait wait
                         :publish-static-tf static-p
                         :parent-frame parent
                         :root-frame root
                         :static-xyz xyz
                         :static-rpy rpy))
    (detachable-tool-clear-arm-state a)
    t))

(defun tool-end-coords (robot arm)
  (or (detachable-tool-state-get (detachable-tool-normalize-arm arm) :tip)
      (send robot (detachable-tool-normalize-arm arm) :end-coords)))

(defun tool-inverse-kinematics (robot arm target-coords &rest args
                                      &key &allow-other-keys)
  (send* robot (detachable-tool-normalize-arm arm) :inverse-kinematics
         target-coords
         :move-target (tool-end-coords robot arm)
         args))

(defun tool-robot-inverse-kinematics (robot arm target-coords &rest args
                                            &key &allow-other-keys)
  (let ((a (detachable-tool-normalize-arm arm)))
    (send* robot :inverse-kinematics
           target-coords
           :move-target (tool-end-coords robot a)
           :link-list (send robot :link-list
                            (send robot a :end-coords :parent)
                            (send robot a :root-link))
           args)))

(defun tool-ik (robot arm target-coords &rest args &key &allow-other-keys)
  (send* robot (detachable-tool-normalize-arm arm) :inverse-kinematics
         target-coords
         :move-target (tool-end-coords robot arm)
         args))

;; Keep the detachable-hand dependency out of mycoboteus itself while allowing
;; the robot model to provide the usual chained message API after mycoboteus is
;; loaded first:
;;   (send *mycobot* :rhand :open-holder :wait t)
(when (boundp 'mycobot-robot)
  (defmethod mycobot-robot
    (:rhand (&rest args)
      (if args
          (send* (rhand-init) args)
        (rhand-init)))))

(provide :dynamixel-detachable-hand-tool-change
         "package://dynamixel_detachable_hand/euslisp/tool-change.l")
