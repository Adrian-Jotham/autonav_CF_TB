#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Callback definition

def active_cb(extra):
    rospy.loginfo("Goal pose processing...")

def feedback_cb(feedback):
    rospy.loginfo("Current location: " + str(feedback))

def done_cb(status, result):
    if status == 3:
        rospy.loginfo("Goal reached.")
    elif status in [2, 8]:
        rospy.loginfo("Goal cancelled.")
    elif status == 4:
        rospy.loginfo("Goal aborted.")

# Initialize the node

rospy.init_node('multiple_goal_navigation')
navclient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
navclient.wait_for_server()

# List of goals
goals = [
    {'x': 3.708, 'y': -1.91, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.008, 'ow': 0.99},
    {'x': 4.8, 'y': -1.88, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.71, 'ow': 0.69},
    # {'x': 4.6, 'y': -2.98, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.76, 'ow': 0.63},
    # {'x': 4.524, 'y': -4.23, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.74, 'ow': 0.666},
    {'x': 4.3, 'y': -4.87, 'z': 0, 'ox': 0, 'oy': 0, 'oz': 0.99, 'ow': 0.089},
    {'x': 0.16, 'y': -2.98, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.2907, 'ow': 0.95},
    {'x': 4.441, 'y': -3.83, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.999, 'ow': 0.001},
   
]

# Iterate through the list of goals
for index, goal_coords in enumerate(goals):
    rospy.loginfo("Sending goal {}...".format(index + 1))
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_coords['x']
    goal.target_pose.pose.position.y = goal_coords['y']
    goal.target_pose.pose.position.z = goal_coords['z']
    goal.target_pose.pose.orientation.x = goal_coords['ox']
    goal.target_pose.pose.orientation.y = goal_coords['oy']
    goal.target_pose.pose.orientation.z = goal_coords['oz']
    goal.target_pose.pose.orientation.w = goal_coords['ow']

    navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
    finished = navclient.wait_for_result()

    if not finished:
        rospy.logerr("Server not available.")
    else:
        # rospy.loginfo("Result: {}".format(navclient.get_result()))
        pass

    rospy.sleep(1)  # Optional: wait before sending the next goal

rospy.loginfo("All goals processed.")
