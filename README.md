# Autonomous-Snowplow-2022-2023
code for 2022-2023 snowplow group.

Sensors in use;
- Trimble GPS reciever, swiftnav antenna, 2 1024 ppr encoders, Intel d435i depth camera

- robot_localization EKF fusing data from the 3 sensors

- RTABmap used for mapping
  - depthimage_to_laserscan package to create scan data to assist obstacle detection and costmap updating
  
  - Voxel_grid nodelet used to filter raw pointcloud2 data from the realsense camera
    - Filtered pointcloud2 published to obstacle_detection node
    - differentiates ground from obstacles
    - Publishes 2D projected obstacles: Costmaps use ground pointcloud data for clearing and /obstacles data for marking, assisted by scans from
    depthimage_to_laserscan for marking (eventually replaced by LiDAR)


