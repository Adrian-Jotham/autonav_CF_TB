#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
import numpy as np
import time

def create_moving_marker(x_start, y_start, x_end, y_end, velocity=0.22):
    """
    Create a moving marker that navigates from start to end coordinates.

    :param x_start: Starting x coordinate
    :param y_start: Starting y coordinate
    :param x_end: Ending x coordinate
    :param y_end: Ending y coordinate
    :param velocity: Velocity of the marker
    """
    rospy.init_node('moving_marker_tb3')
    marker_pub = rospy.Publisher('Turtlebot3_2', Marker, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz update rate

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "moving_marker"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    # Calculate direction vector and normalize it
    direction = np.array([x_end - x_start, y_end - y_start], dtype=float)
    distance = np.linalg.norm(direction)
    direction = direction / distance

    current_position = np.array([x_start, y_start], dtype=float)
    target_position = np.array([x_end, y_end], dtype=float)

    while not rospy.is_shutdown():
        # Update marker position
        marker.pose.position.x = current_position[0]
        marker.pose.position.y = current_position[1]
        marker.pose.position.z = 0.3

        # Publish marker
        marker_pub.publish(marker)

        # Move the marker towards the target position
        current_position += direction * velocity / 10.0

        # Stop if the marker has reached or passed the target position
        if np.linalg.norm(current_position - target_position) <= velocity / 10.0:
            break

        rate.sleep()

if __name__ == '__main__':
    try:
        time.sleep(3)
        create_moving_marker(0, 0, 5, 0)
    except rospy.ROSInterruptException:
        pass
