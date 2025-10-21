#!/usr/bin/env roseus

(require "package://roseus/euslisp/actionlib.l")
(ros::roseus "lhand_action_controller")
(ros::roseus-add-msgs "control_msgs")
(ros::roseus-add-msgs "trajectory_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "actionlib_msgs")

(defclass rhand-interface
  :super propertied-object
  :slots (action-client- move-state- joint-states- groupname))

(defmethod rhand-interface
  (:init (&key ((:groupname grp) "rhand"))
    (setq groupname grp)
    (setq action-client-
          (instance ros::simple-action-client :init
                    "/rhand/position_joint_trajectory_controller/follow_joint_trajectory"
                    control_msgs::FollowJointTrajectoryAction
                    :groupname groupname))
    (unless (send action-client- :wait-for-server 5)
      (ros::ros-warn "Action server not available"))
    (ros::subscribe "/rhand/joint_states" sensor_msgs::JointState
                    #'send self :joint-states-callback 1 :groupname groupname)
    (setq move-state- nil)
    (setq joint-states- (make-hash-table))
    self)

  (:joint-states-callback (msg)
    (let ((names (send msg :name))
          (positions (send msg :position)))
      (dotimes (i (length names))
        (sethash (elt names i) joint-states- (elt positions i)))))

  (:get-joint-state (joint-name)
    (send self :spin-once)
    (gethash joint-name joint-states-))

  (:spin-once () (ros::spin-once groupname))

  (:move-hand (grasp-angle &key (wait t) (tm 1.0) (velocity 0.0) (acceleration 0.0) (effort 0.0))
    (let* ((goal (instance control_msgs::FollowJointTrajectoryActionGoal :init))
           (traj (instance trajectory_msgs::JointTrajectory :init))
           (point (instance trajectory_msgs::JointTrajectoryPoint :init)))
      (send traj :joint_names (list "rhand_joint"))
      (send point :positions (float-vector (float grasp-angle)))
      (send point :velocities (float-vector (float velocity)))
      (send point :accelerations (float-vector (float acceleration)))
      (send point :effort (float-vector (float effort)))
      (send point :time_from_start (ros::time (float tm)))
      (send traj :points (list point))
      (send goal :goal :trajectory traj)
      (send action-client- :send-goal (send goal :goal))
      (when wait
        (send action-client- :wait-for-result :timeout (+ tm 5.0))
        (setq move-state- (send action-client- :get-state))
        (unless (eq move-state- actionlib_msgs::GoalStatus::*succeeded*)
          (ros::ros-warn "Hand movement failed: ~A" move-state-))))
    move-state-)

  (:cancel-move-hand () (send action-client- :cancel-goal))
  (:hand-moving-p () (eq (send action-client- :get-state) actionlib_msgs::GoalStatus::*active*))

  (:open (&key (wait t) (tm 1.0) (velocity 2.0) (acceleration 0.0) (effort 0.0))
    (send self :move-hand 0.0 :wait wait :tm tm :velocity velocity :acceleration acceleration :effort effort))

  (:close (&key (wait t) (tm 1.0) (velocity 2.0) (acceleration 0.0) (effort 0.0))
    (send self :move-hand -2.7 :wait wait :tm tm :velocity velocity :acceleration acceleration :effort effort))

  (:open-holder (&key (wait t) (tm 1.0) (velocity 0.5) (acceleration 0.0) (effort 0.0))
    (send self :move-hand -0.1 :wait wait :tm tm :velocity velocity :acceleration acceleration :effort effort))

  (:close-holder (&key (wait t) (tm 1.0) (velocity 0.5) (acceleration 0.0) (effort 0.0))
    (send self :move-hand 0.08 :wait wait :tm tm :velocity velocity :acceleration acceleration :effort effort))

  
  (:wait-for-hand () (send action-client- :wait-for-result)))

(defun rhand-init ()
  (if (not (boundp '*rhand*))
      (setq *rhand* (instance rhand-interface :init)))
  *rhand*)

(defun rhand (&key grasp (velocity 0.0) (acceleration 0.0) (effort 0.0))
  (unless (boundp '*rhand*) (rhand-init))
  (let ((grasp-angle
         (cond ((eq grasp 'open) 0.0)
               ((eq grasp 'close) -3.0)
               ((numberp grasp) (float grasp))
               (t 0.0))))
    (send *rhand* :move-hand grasp-angle :velocity velocity :acceleration acceleration :effort effort)))
