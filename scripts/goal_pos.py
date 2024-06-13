#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#Callback definition

def active_cb(extra):
    rospy.loginfo("goal pose processing")

def feedback_cb(feedback):
    rospy.loginfo("Current location: " +str(feedback))

def done_cb(status, result):
    if status == 3:
        rospy.loginfo("Reached")
    if status == 2 or status == 8:
        rospy.loginfo("Cancelled")
    if status == 4:
        rospy.loginfo("Aborted")

rospy.init_node('goal_pose')

navclient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
navclient.wait_for_server()

#Navigation goal 1
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()

goal.target_pose.pose.position.x = -0.148
goal.target_pose.pose.position.y = -1.25
goal.target_pose.pose.position.z = 0
goal.target_pose.pose.orientation.x = 0
goal.target_pose.pose.orientation.y = 0
goal.target_pose.pose.orientation.z = -0.704
goal.target_pose.pose.orientation.w = 1

navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
finished = navclient.wait_for_result()

if not finished :
    rospy.logerr("Server not available")
else:
    rospy.loginfo(navclient.get_result)