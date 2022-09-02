#!/usr/bin/env python3
import rclpy

#from time import sleep 

from rclpy.node import Node #used to create nodes
from geometry_msgs.msg import Twist # Linear and angular Vel.
from pyfirmata import Arduino, util


board = Arduino("/dev/ttyACM0")
it = util.Iterator(board)

Lmotor_speed = board.get_pin('d:3:p') # PWM pin 3 to AN1
Lmotor_DIR = board.get_pin('d:4:o') # define LMotor direction->IN1

Rmotor_speed = board.get_pin('d:9:p') # PWM pin 3 to AN2
Rmotor_DIR = board.get_pin('d:8:o') # define RMotor direction-> IN2


def Motor_Signals(v = 0, yaw_rate = 0):
    # linear -.7 to +.7, angular -0.4 to +0.4 
    if v < 0:
        Lmotor_DIR.write(0) # if v is neg, set direction backwards
        Lmotor_speed.write(-v*2)

        Rmotor_DIR.write(0) # if v is neg, set direction backwards
        Rmotor_speed.write(-v)
    else:
        Lmotor_DIR.write(1) # if v is neg, set direction backwards
        Lmotor_speed.write(v*2)

        Rmotor_DIR.write(1) # if v is neg, set direction backwards
        Rmotor_speed.write(v)





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

        Motor_Signals(v, yaw_rate)

        #Planning to use these to calculate the signal to send to motor controller
        # Have some sort of algorithm => as angular z gets higher, lower speed of one motor by that much (Lmotor_Speed = (1-2z))

def main(args=None):
    rclpy.init(args=args) # first line in any ros2 program

   
    node = MyNode()
    



    rclpy.spin(node)

    rclpy.shutdown()


if __name__== "__main__":

    main()