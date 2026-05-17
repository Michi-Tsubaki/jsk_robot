#!/usr/bin/env roseus

(require "package://dynamixel_detachable_hand/euslisp/rhand-interface.l")

(ros::roseus "detacher_test")

(setq *rhand* (rhand-init))

;; attach: clockwise 400 mA for 1 sec, then 0 mA
(send *rhand* :attach :tm 1)

;; detach: counter-clockwise 400 mA for 1 sec, then 0 mA
(send *rhand* :detach :tm 1)

;; explicit stop: 0 mA
(send *rhand* :stop-detach)
