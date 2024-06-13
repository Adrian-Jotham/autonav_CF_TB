#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time

def move_forward():
    # Function to move the Turtlebot forward
    move_cmd.linear.x = 0.2
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)

def stop_robot():
    # Function to stop the Turtlebot
    move_cmd.linear.x = 0
    move_cmd.angular.z = 0
    pub.publish(move_cmd)

def turn_180():
    # Function to turn the Turtlebot 180 degrees
    move_cmd.angular.z = 1.57
    pub.publish(move_cmd)
    rospy.sleep(2)  # Sleep time needed to complete the turn, adjust as necessary
    stop_robot()  # Stop after turning

def move_sequence():
    # This function orchestrates the movement sequence
    move_forward()
    rospy.sleep(10)  # Move forward for 10 seconds
    stop_robot()
    rospy.sleep(1)
    turn_180()
    rospy.sleep(1)

    if not rospy.is_shutdown():
        move_forward()
        rospy.sleep(10)
        stop_robot()
        rospy.sleep(1)
        turn_180()
        rospy.sleep(1)

def start_moving():
    rospy.loginfo("Starting lawnmower pattern movement")
    move_sequence()

def shutdown_handler():
    rospy.loginfo("Shutting down lawnmower pattern")
    stop_robot()

if __name__ == '__main__':
    try:
        rospy.init_node('turtlebot_lawnmower', anonymous=True)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        move_cmd = Twist()

        rospy.on_shutdown(shutdown_handler)  # Set the function to be called on node shutdown
        start_moving()
    except rospy.ROSInterruptException:
        pass
