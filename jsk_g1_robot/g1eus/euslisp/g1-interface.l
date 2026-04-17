(load "package://pr2eus/robot-interface.l")
(require :g1 "package://g1eus/euslisp/g1.l")

(defclass g1-interface
  :super robot-interface
  :slots ())

(defmethod g1-interface
  (:init (&rest args)
    (send-super :init :robot g1-robot)))

(defun g1-init ()
  (unless (boundp '*ri*)
    (setq *ri* (instance g1-interface :init)))
  (unless (boundp '*g1*)
    (setq *g1* (instance g1-robot :init))))
