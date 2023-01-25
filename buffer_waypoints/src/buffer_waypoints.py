#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from move_base_msgs.msg import MoveBaseAction,MoveBaseActionGoal, MoveBaseGoal, MoveBaseActionResult
import actionlib
import time

# Create a queue to hold the waypoints
waypoint_queue = []

# Flag to indicate whether the first waypoint has been reached
first_waypoint_reached = False

# Function to be called after each waypoint is reached
def pause_for_4_seconds():
    # Pause for 4 seconds
    time.sleep(2)

# Function to be called after the first waypoint is reached
def move_plow():
    rospy.loginfo("plow moving")
    # Implement the code to move the plow here
    pass

# Callback function for the /move_base/goal topic
def goal_callback(goal_msg):
    global first_waypoint_reached
    # Add the waypoint to the queue
    waypoint_queue.append(goal_msg)
    if not first_waypoint_reached and len(waypoint_queue) == 1:
        # The first waypoint has been reached
        first_waypoint_reached = True
        # Call the function to move the plow
        move_plow()

# Callback function for the /move_base/result topic
def result_callback(result_msg):
    global first_waypoint_reached
    if len(waypoint_queue) > 1 or (first_waypoint_reached and len(waypoint_queue) == 1):
        # Call the function to pause for 4 seconds
        pause_for_4_seconds()

# Callback function for the /clicked_point topic
def clicked_point_callback(clicked_point_msg):
    # Convert the clicked point to a PoseWithCovarianceStamped message
    ##goal_msg = PoseWithCovarianceStamped()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position = clicked_point_msg.pose.pose.position
    goal.target_pose.pose.orientation = clicked_point_msg.pose.pose.orientation
    #goal_msg.target_pose. = [0] * 36
    # Add the converted message as a waypoint to the queue
    rospy.loginfo("waypoint added")
    waypoint_queue.append(goal)



# Main function
def main():
    # Initialize the ROS node
    rospy.init_node('buffer_waypoints')

    # Subscribe to the /move_base/goal topic
    goal_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, goal_callback)

    # Subscribe to the /move_base/result topic
    result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, result_callback)

    # Subscribe to the /clicked_point topic
    clicked_point_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, clicked_point_callback)

    # Create an action client for the /move_base action
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    client.wait_for_server()

    # Loop at a fixed rate
    rate = rospy.Rate(10)


    while not rospy.is_shutdown():
        # Check if there are any waypoints in the queue

        # take user input: Y to start nav
        #user_input = input("press Y to start nav: ")
        time.sleep(15)
        #if user_input == "Y":
            #pass
        """
        while len(waypoint_queue) > 0:
            # Get the next waypoint from the queue
            global first_waypoint_reached
            #status = client.get_state()
            #if first_waypoint_reached == False:
                #time.sleep(5)
            goal = waypoint_queue.pop()
            # Send the waypoint to the move_base action server           
            client.send_goal(goal)
            rospy.loginfo("sent: ")
            client.wait_for_result()
            rospy.loginfo("waypoint reached?")
        """
        for goal in waypoint_queue:
            
            # Send the waypoint to the move_base action server           
            client.send_goal(goal)
            rospy.loginfo("sent: ")
            client.wait_for_result()
            rospy.loginfo("waypoint reached?")
    rospy.loginfo("waypoints reached! ")
    rate.sleep()

if __name__ == '__main__':

    main()
