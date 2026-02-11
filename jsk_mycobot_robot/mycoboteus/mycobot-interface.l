#!/usr/bin/env roseus

(require "package://pr2eus/robot-interface.l")
(load "package://mycoboteus/mycobot-utils.l")
(ros::roseus-add-srvs "mycobot_communication")

(defclass mycobot-interface
  :super robot-interface
  :slots ()
  )

(defmethod mycobot-interface
  (:init (&rest args)
	 (send-super* :init :robot mycobot-robot :joint-states-topic "joint_states" args)
	 )
  (:angle-vector (av time &key (wait t))
		 (ros::service-call
		  "/set_joint_angles"
		  (instance mycobot_communication::SetAnglesRequest
			    :init
			    :joint_1 (elt av 0)
			    :joint_2 (elt av 1)
			    :joint_3 (elt av 2)
			    :joint_4 (elt av 3)
			    :joint_5 (elt av 4)
			    :joint_6 (elt av 5)
			    :speed (min 100 (max 1  (- 111 (* 11/1000 time))))))
		 (when wait
		   (unix:sleep 1))
		 t)
  )

(defun mycobot-init ()
  (setq *mycobot* (instance mycobot-robot :init))
  (setq *robot* *mycobot*)
  (setq *ri* (instance mycobot-interface :init))
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (objects (list *robot*))
  )
