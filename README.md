# poshold-quadcopter
this repo contains sourcecode for postiion hold of quadcopter drone in ROS2 environment and ardupilot firmware FCU. It works by using velocity data from MAVLink, compare it with setpoint (0) and send control command to maintain its position.
environment : ROS2 humble
FCU firmware: ardupilot 4.x.x
note : untested :D

main code directory: src/stabilization/stabilization/main.py
