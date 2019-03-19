#!/usr/bin/env roseus

(ros::roseus "debug")
(load "package://pr2eus/speak.l")

(do-until-key
 (speak-en "hello")
 (unix:sleep 1)
 )
