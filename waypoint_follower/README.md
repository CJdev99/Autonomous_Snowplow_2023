# waypoint_follower
## This node serves as a controller for the rest of our robot. It is a workaround to a move_base bug where there is never a response when a waypoint is successfully reached. It also serves as an interface between the ROS robot and the web control client, where the website publishes to the topics below to control navigation modes.
### Subscriptions:
* /start_navigation: When triggered, it starts popping waypoints from the queue; when triggered again, it cancels the current waypoint and stops navigation
* /save_waypoint: gets the current pose of robot w.r.t the /map frame, and saves it to the waypoint queue
* /initialpose: Saves clicked waypoint from rviz
### Publishers:
* /cancel_waypoint: Cancels current goal sent to move_base

