# Gnss-IMU_Fusion
ros2 version 

using ESKF

you can set you own parameters in include/gins.h  GinsNode's constructor (mainly is the I_p_Gps,it be acquiescently set to {0,0,0})

# required
ros2

Eigen

glog (version 0.4.0) find it on the below LINKS:
https://github.com/google/glog/releases

# Notes
for getting GNSS
you need install `nmea-navsat-driver`  `nmea-navsat-msgs`

# run
---
>colcon build
 
  
