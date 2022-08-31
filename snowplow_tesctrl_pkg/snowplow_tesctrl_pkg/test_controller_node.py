#!/usr/bin/env python3
import rclpy

#from time import sleep 

from rclpy.node import Node #used to create nodes
from geometry_msgs.msg import Twist # Linear and angular Vel.

class MyNode(Node):

    def __init__(self):
        super().__init__("test_controller")
        self.get_logger().info("Hello ROS2")

        #Code goes here
        # Create a subscriber 
        # This node subscribes to messages of type 
        # geometry_msgs/Twist.msg. We are listening to the velocity commands here.
        # The maximum number of queued messages is 10.
        self.velocity_subscriber = self.create_subscription(
                               Twist,
                               '/cmd_vel',
                               self.velocity_callback,
                               10)

    def velocity_callback(self, msg):
        """
        Listen to the velocity commands (linear forward velocity 
        in the x direction in the robot's reference frame and 
        angular velocity (yaw rate) around the robot's z-axis.
        [v,yaw_rate]
        [meters/second, radians/second]
         """

        # Forward velocity in the robot's reference frame
        v = msg.linear.x
        
        # Angular velocity around the robot's z axis
        yaw_rate = msg.angular.z

        #Planning to use these to calculate the signal to send to motor controller
        # Have some sort of algorithm => as angular z gets higher, lower speed of one motor by that much (Lmotor_Speed = (1-2z))

def main(args=None):
    rclpy.init(args=args) # first line in any ros2 program

   
    node = MyNode()
    



    rclpy.spin(node)

    rclpy.shutdown()


if __name__== "__main__":

    main()