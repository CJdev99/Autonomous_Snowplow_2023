#!/usr/bin/env python3
import rclpy

#from time import sleep 

from rclpy.node import Node #used to create nodes
from geometry_msgs.msg import Twist # Linear and angular Vel.
from pyfirmata import Arduino, util


board = Arduino("/dev/ttyACM0") # Access board via serial
it = util.Iterator(board) # Used to throttle signal when board is overloaded (usually only for analog)

Lmotor_speed = board.get_pin('d:3:p') # PWM pin 3 to AN1
Lmotor_DIR = board.get_pin('d:4:o') # define LMotor direction->IN1

Rmotor_speed = board.get_pin('d:9:p') # PWM pin 3 to AN2
Rmotor_DIR = board.get_pin('d:8:o') # define RMotor direction-> IN2

# Function that writes PWM/DC signal to arduino pins, with specified signal
# Eventually will have node for each motor, when PID controller is implemented
def Motor_Signals(v = 0, yaw_rate = 0):
    # linear v, angV ranges from -.7 to +.7, angular -0.4 to +0.4 respectively
    Lmotor_calc = abs(v)-yaw_rate/2
    if Lmotor_calc <= 0:
            Lmotor_calc = 0
        elif Lmotor_calc >= 200: # check if 200 is max
            Lmotor_calc = 200
            
     Rmotor_calc = abs(v)+yaw_rate/2
        if Rmotor_calc <= 0:
            Rmotor_calc = 0
        elif Rmotor_calc >= 200: # check if 200 is max
            Rmotor_calc = 200
            
            
    if v < 0:
        Lmotor_DIR.write(0) # if v is neg, set digital write low
       
        Lmotor_speed.write(Lmotor_calc) # joystick to the right, yaw rate will be (-), and we want the right motor to slow
        # Joystick to the left, yaw_rate will
        # Will have to tune yaw_rate multiplier thru testing
        
        Rmotor_DIR.write(0)
        Rmotor_speed.write(Rmotor_calc) 
    else:
        Lmotor_DIR.write(1) # if v is neg, set direction backwards
        Lmotor_speed.write(Lmotor_calc)

        Rmotor_DIR.write(1) # if v is neg, set direction backwards
        Rmotor_speed.write(Rmotor_calc)





class MyNode(Node):

    def __init__(self):
        super().__init__("test_controller") 
        self.get_logger().info("TestController Node is Running") # Prints message to command line

        #
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
        #Call motor function, which sends the appropriate signal to the pins
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
