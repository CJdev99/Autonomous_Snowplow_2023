# Autonomous-Snowplow-2022-2023
Metapackage including packages, configurations, and launch files used on the 2022-2023 autonomous snowplow project.

Sensors in use:
- Trimble GPS reciever, swiftnav antenna, 2 1024 ppr encoders, Intel d435i depth camera

- robot_localization EKF fusing data from the 3 sensors
  - Wheel odometry stream from motor controller driver
  - IMU data stream from openzen_ros C++ driver
    - Filtered & Fused orientation from magnetometer, quaternion, gyro, and accelerometer data using IMU madgwick filter
  - GPS data stream from nmea_socket_pub, parsed GPS data converted to robot coordinate frame using navsat_transform
 
 Intel D435i RGB-D camera & SiCK LiDAR used in RTABmap package for SLAM/mapping
  - Parameters and filters located under (Autonomous_Snowplow_2023/jetson_snowplow/navstack_pub/launch/voxelgrid_throt.launch)
  - RGBD_Sync nodelet used to synchronize frames & data stream from realsense driver
  - points_xyzrgb creates pointcloud2 data from synchronized RGB-D images, used for 3-D voxel obstacle layer
  - obstacle_detection package used to differentiate ground & obstacles, used for clearing & marking in costmaps
  - RTABmap_ros used to generate & store grid map from RGB-D images, will include LiDAR for mapping when functional
 
 Buffer_waypoints package uses move_base action servers to buffer waypoints sent to the node as pose goals
  - subscribes to /initialpose topic published by RVIZ, allowing user to click desired location & orientation for the robot to travel
 
 nmea_socket_pub:
  - reads GPS/GNSS NMEA sentence streams over specified TCP port, parsing GGA & GST sentences to both applicable GPS fix data, as well as a published covariance matrix
    - covariance matrix updated in real-time, so the kalman filter fused data takes more or less accurate GPS data into account

