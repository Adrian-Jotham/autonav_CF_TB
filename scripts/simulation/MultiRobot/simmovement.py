#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tf.transformations
import random
import math

def calculate_yaw(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1)

def send_goal(robot_namespace, x, y, yaw):
    goal_pub = rospy.Publisher('/{}/move_base_simple/goal'.format(robot_namespace), PoseStamped, queue_size=10)
    rospy.sleep(1)  # Give the publisher time to connect

    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = 'map'
    
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0.0
    
    # Convert yaw to quaternion
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.pose.orientation.x = q[0]
    goal.pose.orientation.y = q[1]
    goal.pose.orientation.z = q[2]
    goal.pose.orientation.w = q[3]

    goal_pub.publish(goal)
    rospy.loginfo('Sent goal to {}: x={}, y={}, yaw={}'.format(robot_namespace, x, y, yaw))

def distance_between_points(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def wait_for_goal_reached(robot_namespace, goal_position, threshold=0.5, timeout=30):
    pose_topic = '/{}/amcl_pose'.format(robot_namespace)
    start_time = rospy.Time.now()
    rate = rospy.Rate(2)  # Check at 2 Hz
    while not rospy.is_shutdown():
        try:
            current_pose = rospy.wait_for_message(pose_topic, PoseWithCovarianceStamped, timeout=5)
            current_position = (current_pose.pose.pose.position.x, current_pose.pose.pose.position.y)
            if distance_between_points(current_position, goal_position) < threshold:
                rospy.loginfo('{} reached the goal at position: {}'.format(robot_namespace, current_position))
                return True
        except rospy.ROSException:
            pass

        if (rospy.Time.now() - start_time).to_sec() > timeout:
            rospy.loginfo('{} failed to reach the goal within timeout'.format(robot_namespace))
            return False
        rate.sleep()

def choose_new_positions(coordinates, visited_positions, min_distance=3.0):
    potential_positions = [
        (p0, p1) for p0 in coordinates for p1 in coordinates
        if (p0 not in visited_positions and p1 not in visited_positions)
    ]

    # Prioritize positions with minimum distance
    prioritized_positions = [
        (p0, p1) for p0, p1 in potential_positions
        if distance_between_points(p0, p1) > min_distance
    ]

    if prioritized_positions:
        return random.choice(prioritized_positions)

    # If no positions meet the distance criteria, return from potential positions
    return random.choice(potential_positions)

if __name__ == '__main__':
    try:
        rospy.init_node('send_goals', anonymous=True)

        # Coordinates based on provided data and valid map boundaries
        coordinates = [
            (0, 0), (0, -1), (0, -2), (0, -3), 
            (1, 0), (1, -1), (1, -2), (1, -3), (1, -3.5), 
            (2, 0), (2, -1), (2, -2), (2, -3), (2, -4),  
            (3, 0), (3, -1), (3, -3.4), (3, -4), (3, -5), 
            (4, 0), (4, -1), (4, -2), (4, -3), (4, -4),
            (5,0), (5,-1),
            # Removed (5, 1) as it is outside the map boundary
        ]

        # Initial positions and yaw
        tb3_0_position = random.choice(coordinates)
        tb3_1_position = random.choice(coordinates)
        tb3_0_yaw = 0
        tb3_1_yaw = 0

        visited_positions = {tb3_0_position, tb3_1_position}

        while not rospy.is_shutdown():
            # Select new positions ensuring a minimum distance between them and no redundancy
            new_tb3_0_position, new_tb3_1_position = choose_new_positions(coordinates, visited_positions)

            # Calculate new yaws
            tb3_0_yaw = calculate_yaw(tb3_0_position[0], tb3_0_position[1], new_tb3_0_position[0], new_tb3_0_position[1])
            tb3_1_yaw = calculate_yaw(tb3_1_position[0], tb3_1_position[1], new_tb3_1_position[0], new_tb3_1_position[1])

            # Send goals
            send_goal('tb3_0', new_tb3_0_position[0], new_tb3_0_position[1], tb3_0_yaw)
            send_goal('tb3_1', new_tb3_1_position[0], new_tb3_1_position[1], tb3_1_yaw)

            # Wait for both robots to reach their goals
            tb3_0_reached = wait_for_goal_reached('tb3_0', new_tb3_0_position)
            tb3_1_reached = wait_for_goal_reached('tb3_1', new_tb3_1_position)

            # Update current positions
            if tb3_0_reached:
                tb3_0_position = new_tb3_0_position
                visited_positions.add(tb3_0_position)
            if tb3_1_reached:
                tb3_1_position = new_tb3_1_position
                visited_positions.add(tb3_1_position)

            # Optional: wait before sending the next goal
            # rospy.sleep(5)
    except rospy.ROSInterruptException:
        pass
