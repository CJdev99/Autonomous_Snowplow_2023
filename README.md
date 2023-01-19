# Autonomous-Snowplow-2022-2023
code for 2022-2023 snowplow group.

Sensors in use:
- Trimble GPS reciever, swiftnav antenna, 2 1024 ppr encoders, Intel d435i depth camera

- robot_localization EKF fusing data from the 3 sensors
  - Wheel odometry stream from motor controller driver
  - IMU data stream from openzen_ros C++ driver
    - Filtered & Fused orientation from magnetometer, quaternion, gyro, and accelerometer data using IMU madgwick filter
 - GPS data stream from nmea_socket_pub, parsed GPS data converted to robot coordinate frame using navsat_transform
 
 
- RTABmap used for mapping


