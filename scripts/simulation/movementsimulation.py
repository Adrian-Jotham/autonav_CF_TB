#!/usr/bin/env python

import rospy
import actionlib
import csv
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped
from datetime import datetime

# Global list to store data
data_list = []
current_pose = None

# Callbacks for waypoint navigation
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

# Callback for gas concentration data
def gas_concentration_callback(msg):
    global data_list
    sec = rospy.get_rostime().secs
    nsec = rospy.get_rostime().nsecs
    data_list.append([sec, nsec, current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z, msg.data])

# Callback for gas concentration data
# def gas_concentration_callback(msg):
#     global data_list, current_pose
#     if current_pose is None or not hasattr(current_pose, 'pose'):
#         rospy.logwarn("Current pose not yet available or incomplete.")
#         return
#     now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
#     data_list.append([now, current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, msg.data])


# Callback for updating the robot's current pose
def current_pose_callback(msg):
    global current_pose
    current_pose = msg

# Initialize the node
rospy.init_node('multiple_goal_navigation')
navclient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
navclient.wait_for_server()

# Subscribe to the mq135/ppm topic and amcl_pose
#rospy.Subscriber('/tb3/mq135/ppm', Float32, gas_concentration_callback)
rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, current_pose_callback)

# List of goals
goals = [
    #all wap
    # {'x': 0.008, 'y': 0.1904, 'z': 0, 'ox': 0, 'oy': 0, 'oz': 0, 'ow': 0.999},
    {'x': 5.253, 'y': -0.1364, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.862, 'ow': 0.506},
    {'x': 5.085, 'y': -0.993, 'z': 0, 'ox': 0, 'oy': 0, 'oz': 0.9997142, 'ow': 0.0509},
    # {'x': 2.6917, 'y': -0.7537, 'z': 0, 'ox': 0, 'oy': 0, 'oz': 0.998, 'ow': 0.0239},
    {'x': -0.0229, 'y': -0.738, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.665, 'ow': 0.746},
    {'x': -0.0132, 'y': -1.324, 'z': 0, 'ox': 0, 'oy': 0, 'oz': 0.0399, 'ow': 0.999},
    {'x': 4.954, 'y': -1.631, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.7907, 'ow': 0.612},
    {'x': 4.524, 'y': -2.876, 'z': 0, 'ox': 0, 'oy': 0, 'oz': 0.982, 'ow': 0.1885},
    {'x': 3.446, 'y': -2.724, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.716, 'ow': 0.6972},
    {'x': 3.3021, 'y': -3.6841, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.053, 'ow': 0.998},
    {'x': 3.987, 'y': -3.758, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.875, 'ow': 0.4828},
    {'x': 3.438, 'y': -5.048, 'z': 0, 'ox': 0, 'oy': 0, 'oz': 0.967, 'ow': 0.251},
    {'x': -0.275, 'y': -2.923, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.0664, 'ow': 0.9977},
    {'x': 2.112, 'y': -2.950, 'z': 0, 'ox': 0, 'oy': 0, 'oz': 0.7432, 'ow': 0.669},
    {'x': 2.136, 'y': -2.296, 'z': 0, 'ox': 0, 'oy': 0, 'oz': 0.998, 'ow': 0.0528},
    {'x': 0.038, 'y': -1.974, 'z': 0, 'ox': 0, 'oy': 0, 'oz': 0.998, 'ow': 0.0598},


    #luardalam
    # {'x': 5.453, 'y': -0.1364, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.862, 'ow': 0.506},
    # {'x': 3.438, 'y': -5.048, 'z': 0, 'ox': 0, 'oy': 0, 'oz': 0.967, 'ow': 0.251},
    # {'x': -0.275, 'y': -2.923, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.0664, 'ow': 0.9977},
    # {'x': 0.008, 'y': 0.1904, 'z': 0, 'ox': 0, 'oy': 0, 'oz': 0, 'ow': 0.999},

    #kiri kanan
    # {'x': 5.453, 'y': -0.1364, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.862, 'ow': 0.506},
    # {'x': 5.085, 'y': -0.793, 'z': 0, 'ox': 0, 'oy': 0, 'oz': 0.9997142, 'ow': 0.0509},
    # {'x': 2.6917, 'y': -0.7537, 'z': 0, 'ox': 0, 'oy': 0, 'oz': 0.998, 'ow': 0.0239},
    # {'x': -0.0229, 'y': -0.538, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.665, 'ow': 0.746},
    # {'x': -0.0132, 'y': -1.324, 'z': 0, 'ox': 0, 'oy': 0, 'oz': 0.0399, 'ow': 0.999},
    # {'x': 4.954, 'y': -1.631, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.7907, 'ow': 0.612},
]

# goals = [
#     # {'x': -0.066, 'y': 0.079, 'z': 0, 'ox': 0, 'oy': 0, 'oz': 0, 'ow': 1},
#     # {'x': 5.3, 'y': 0.55, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.73, 'ow': 0.67},
#     {'x': 5.11, 'y': -0.01, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.99, 'ow': 0.076},
#     {'x': -0.12, 'y': -0.6, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.73, 'ow': 0.67},
#     {'x': -0.05, 'y': -1.17, 'z': 0, 'ox': 0, 'oy': 0, 'oz': 0.106, 'ow': 0.99},
#     {'x': 5, 'y': -0.59, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.781, 'ow': 0.623},
#     {'x': 5, 'y': -1.25, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.997, 'ow': 0.069},
#     {'x': -0.072, 'y': -1.703, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.004, 'ow': 0.999},
#     {'x': 2.53, 'y': -2.167, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.9809, 'ow': 0.1944},
#     {'x': 0.041, 'y': -2.4, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.088, 'ow': 0.99},
#     {'x': 3.9, 'y': -3.08, 'z': 0, 'ox': 0, 'oy': 0, 'oz': 0.7833, 'ow': 0.621},
#     {'x': 3.872, 'y': -1.91, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.008, 'ow': 0.99},
#     {'x': 4.8, 'y': -1.88, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.71, 'ow': 0.69},
#     {'x': 4.3, 'y': -4.87, 'z': 0, 'ox': 0, 'oy': 0, 'oz': 0.99, 'ow': 0.089},
#     {'x': 0.16, 'y': -2.98, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.2907, 'ow': 0.95},
#     # {'x': -0.066, 'y': 0.079, 'z': 0, 'ox': 0, 'oy': 0, 'oz': 0, 'ow': 1},
#     {'x': 3.96, 'y': -3.71, 'z': 0, 'ox': 0, 'oy': 0, 'oz': -0.11, 'ow': 0.99},
# ]

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
        rospy.loginfo("Result: {}".format(navclient.get_result()))

    rospy.sleep(1)  # Optional: wait before sending the next goal

rospy.loginfo("All goals processed and data saved to CSV.")