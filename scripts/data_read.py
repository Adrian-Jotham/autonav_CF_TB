#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from std_msgs.msg import Float32
import time

class GasMarkerPublisher:
    def __init__(self):
        self.pub_gas = rospy.Publisher('tb3/gas_marker_tb', MarkerArray, queue_size=50)
        self.marker_id = 0
        self.robot_pose = Point()
        rospy.Subscriber('/tb3/mq135/ppm', Float32, self.gas_concentration_callback)
        rospy.Subscriber('/tb3/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)

    def get_color_for_concentration(self, concentration):
        if concentration < 300:
            return ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green
        elif concentration < 600:
            return ColorRGBA(1.0, 1.0, 0.0, 1.0)  # Yellow
        else:
            return ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red

    def publish_gas(self, concentration):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "tb3/map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "tb3/gas_concentration_tb"
        marker.id = self.marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = self.robot_pose
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1
        marker.color = self.get_color_for_concentration(concentration)
        marker.lifetime = rospy.Duration()

        marker_text = Marker()
        marker_text.header = marker.header
        marker_text.ns = "tb3/gas_concentration_text"
        marker_text.id = self.marker_id + 10000
        marker_text.type = Marker.TEXT_VIEW_FACING
        marker_text.action = Marker.ADD
        marker_text.pose.position = Point(self.robot_pose.x, self.robot_pose.y, self.robot_pose.z + 0.2)
        marker_text.scale.z = 0.05
        marker_text.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        marker_text.text = "Conc: {:.2f}".format(concentration)
        marker_text.lifetime = rospy.Duration()

        marker_array.markers.append(marker)
        marker_array.markers.append(marker_text)
        
        global then
        now = rospy.get_time()
        if (now - then) > 1.0:
            self.pub_gas.publish(marker_array)
            self.marker_id += 1
            then = now
    

    def amcl_pose_callback(self, msg):
        self.robot_pose.x = msg.pose.pose.position.x
        self.robot_pose.y = msg.pose.pose.position.y
        self.robot_pose.z = msg.pose.pose.position.z

    def gas_concentration_callback(self, msg):
        self.publish_gas(msg.data)

if __name__ == "__main__":
    rospy.init_node('gas_marker_tb')
    then =rospy.get_time()
    gas_marker_publisher = GasMarkerPublisher()
    rospy.spin()
    then =rospy.get_time()
