# MSU, Mankato's 2023 Autonomous Snowplow

![](https://github.com/CJdev99/Autonomous_Snowplow_2023/blob/main/robot_gif.gif)

[YT video demo](https://www.youtube.com/watch?v=lg9_K-PXslY&ab_channel=ChaseDevitt)


Metapackage including ROS packages, configurations, and launch files used on the 2022-2023 MNSU autonomous snowplow project.

Sensors in use:
- Trimble GPS reciever, swiftnav antenna, LPMS U3 IMU, 2 1024 ppr encoders, Intel d435i RGB-D camera

- robot_localization EKF fusing data from the 3 sensors
  - Wheel odometry stream from roboteq_diff_driver - accesses motor controller through C++ api to  receive encoder tick count, current in Amps through each motor channel, and temperature of motor controller
  - IMU data stream from openzen_ros C++ driver
    - Filtered & Fused orientation from magnetometer, gyro, and accelerometer data using IMU madgwick filter
  - GPS data stream from nmea_socket_pub, parsed NMEA sentences converted to robot coordinate frame using navsat_transform
 
 Intel D435i RGB-D camera used in RTABmap package for SLAM/mapping
  - Parameters and filters located under [here](Autonomous_Snowplow_2023/jetson_snowplow/navstack_pub/launch/voxelgrid_throt.launch)
    Image Pipeline:
  - RGBD_Sync nodelet: Synchronize camera messages, throttle data to improve SLAM performance 
  - points_xyzrgb creates pointcloud2 data from synchronized RGB-D images, used for 3-D voxel obstacle layer
  - obstacle_detection package used to differentiate ground & obstacles, used for clearing & marking in costmaps
  - RTABmap_ros used to generate & store grid map from RGB-D images, will include LiDAR for mapping when functional
 
 **Buffer_waypoints** package is a move_base action client that allows following of multiple waypoints; will be upgraded to state machine/behavior tree in future
  - subscribes to /initialpose topic published by RVIZ, adds clicked poses from that topic to a waypoint queue & begins navigating when prompted
  
 **Waypoint_follower** node is similar to buffer_waypoints but includes functionality with the react web app & a workaround to move_base not changing status when waypoint is reached
 
 [Web Interface: ](https://github.com/CJdev99/Snowplow_web_interface)  React App that allows teleop, video stream, and autonomous functionality
 
 **nmea_socket_pub:**
  - reads GPS/GNSS NMEA sentence streams over specified TCP port, parsing GGA & GST sentences to both applicable GPS fix data, as well as a published covariance matrix
    - covariance matrix updated in real-time, so the kalman filter fused data takes more or less accurate GPS data into account
    
 OpenCV scripts used to convert PNG images of a map/blueprint into binary for use in map servers.

## Operation
On machine running ROS master:

    roslaunch navstack_pub camera.launch
    rosrun web_video_server web_video_server
    
On rpi running web server & website client node (on same ROS master):

    rosrun waypoint_follower driver
    roslaunch rosbridge_server rosbridge_server.launch
    serve -s build #(in react app folder; If apache not configured & react app is built)
    *(to access from other network, configure husarnet and join p2p network)
