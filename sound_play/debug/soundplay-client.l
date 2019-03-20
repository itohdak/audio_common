#!/usr/bin/env roseus

(load "package://pr2eus/speak.l")

(ros::roseus "soundplay_client")

(defun main
    ()
  (setq *i* 0)
  (setq *sleep-time* (ros::get-param "~sleep_time" 1))
  (do-until-key
   (incf *i*)
   (speak-en (format nil "test~a" *i*))
   (unix:usleep (round (* *sleep-time* 1e6)))
   )
  )
(main)
