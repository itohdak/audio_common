#!/usr/bin/env roseus

(load "package://pr2eus/speak.l")

(ros::roseus "soundplay_client")

(defun main
    ()
  (setq *i* 0)
  (setq *sleep-time* (ros::get-param "~sleep_time" 1))
  (setq *la* (intern (string-upcase (ros::get-param "~language" "en")) *keyword-package*))
  ;; (setq *la* :en)
  ;; (setq *la* :jp)
  (do-until-key
   (incf *i*)
   (cond
    ((eq *la* :jp)
     (print (format nil "テスト~a" *i*))
     (speak-jp (format nil "テスト~a" *i*))
     )
    (t
     (print (format nil "test~a" *i*))
     (speak-en (format nil "test~a" *i*))
     ))
   (unix:usleep (round (* *sleep-time* 1e6)))
   )
  )
(main)
