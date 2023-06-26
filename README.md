# Imu and Uwb Fusion by ESKF ver1.0
This repository is Error-State Kalman Filter for Imu-Uwb sensor fusion.
This is modified version of chennuo0125-HIT/imu_gps_fusion repository.
## Explain
- [https://blog.csdn.net/weixin_37835423/article/details/109452148](https://blog.csdn.net/weixin_37835423/article/details/109452148)
- [https://blog.csdn.net/weixin_37835423/article/details/109476346](https://blog.csdn.net/weixin_37835423/article/details/109476346)

## Requirements

- [ROS Noetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- Eigen3

## Reference Theory

- [Quaternion kinematics for error state kalman filter](http://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf)

## Used in

- [wgs_conversions]( https://github.com/gyjun0230/wgs_conversions ) for transform between coordinate in wgs frame and coordinate in enu frame


