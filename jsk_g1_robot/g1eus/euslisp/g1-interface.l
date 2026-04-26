(load "package://pr2eus/robot-interface.l")
(require :g1 "package://g1eus/euslisp/g1.l")

(defclass g1-interface
  :super robot-interface
  :slots ())

(defmethod g1-interface
  (:upper-body-controller
   ()
   (list
    (list
     (cons :controller-action "upper_body_controller/follow_joint_trajectory")
     (cons :controller-state "upper_body_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names    (list
			    "waist_yaw_joint"
			    "waist_roll_joint"
			    "waist_pitch_joint"
			    "left_shoulder_pitch_joint"
			    "left_shoulder_roll_joint"
			    "left_shoulder_yaw_joint"
			    "left_elbow_joint"
			    "left_wrist_roll_joint"
			    "left_wrist_pitch_joint"
			    "left_wrist_yaw_joint"
			    "right_shoulder_pitch_joint"
			    "right_shoulder_roll_joint"
			    "right_shoulder_yaw_joint"
			    "right_elbow_joint"
			    "right_wrist_roll_joint"
			    "right_wrist_pitch_joint"
			    "right_wrist_yaw_joint")))))

  (:default-controller
   ()
   (append
    (send self :upper-body-controller)
    )
   )

  (:init (&rest args)
    (send-super* :init :robot g1-robot args)))

(defun g1-init ()
  (unless (boundp '*ri*)
    (setq *ri* (instance g1-interface :init)))
  (unless (boundp '*g1*)
    (setq *g1* (instance g1-robot :init))))
