#!/usr/bin/env python

import rospy
import actionlib
import csv
import random
import atexit
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped
from datetime import datetime

# Global list to store data
data_list = []
current_pose = None
visited_coords = set()

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

# Callback for updating the robot's current pose
def current_pose_callback(msg):
    global current_pose
    current_pose = msg

# Writing to CSV file
def write_to_csv():
    with open("/home/dryan/Desktop/TBCFRANDOM/tb3.csv", "w") as file:
        writer = csv.writer(file)
        writer.writerow(['sec', 'nsec', 'X', 'Y', 'Z', 'Concentration'])
        for data in data_list:
            writer.writerow(data)
    rospy.loginfo("Data saved to CSV.")

# Register write_to_csv to be called on program exit
atexit.register(write_to_csv)

# Initialize the node
rospy.init_node('multiple_goal_navigation')
navclient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
navclient.wait_for_server()

# Subscribe to the mq135/ppm topic and amcl_pose
rospy.Subscriber('/tb3/mq135/ppm', Float32, gas_concentration_callback)
rospy.Subscriber('/tb3/amcl_pose', PoseWithCovarianceStamped, current_pose_callback)

# List of coordinates
coordinates = [
    (0.0, -1.0), (0.0, -2.0), 
    (1.0, 0.0), (1.0, -1.0), (1.0, -2.0), (1.0, -3.0), (1.0, -3.5), 
    (2.0, 0.0), (2.0, -1.0), (2.0, -2.0), (2.0, -4.0),  
    (3.0, 0.0), (3.0, -1.0), (3.0, -3.4), (3.0, -4.0), (3.0, -5.0), 
    (4.0, 0.0), (4.0, -1.0), (4.0, -2.0), (4.0, -3.0), (4.0, -4.0),
    (5.0, 0.0), (5.0, -1.0)
]

goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "tb3/map"
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = 2.0
goal.target_pose.pose.position.y = -2.923
goal.target_pose.pose.position.z = 0
goal.target_pose.pose.orientation.x = 0
goal.target_pose.pose.orientation.y = 0
goal.target_pose.pose.orientation.z = -0.0664
goal.target_pose.pose.orientation.w = 0.9977
navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
finished = navclient.wait_for_result()

if not finished:
    rospy.logerr("Server not available.")
else:
    rospy.loginfo("Result: {}".format(navclient.get_result()))


# Shuffle the coordinates
random.shuffle(coordinates)

# Iterate through the shuffled coordinates
for index, (x, y) in enumerate(coordinates):
    if (x, y) in visited_coords:
        continue
    visited_coords.add((x, y))
    rospy.loginfo("Sending goal {}...".format(index + 1))
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "tb3/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = 0
    goal.target_pose.pose.orientation.w = 1

    navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
    finished = navclient.wait_for_result()

    if not finished:
        rospy.logerr("Server not available.")
    else:
        rospy.loginfo("Result: {}".format(navclient.get_result()))

    # rospy.sleep(1)  # Optional: wait before sending the next goal

rospy.loginfo("All goals processed.")
