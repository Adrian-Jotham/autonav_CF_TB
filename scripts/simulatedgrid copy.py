#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

def create_marker_array(x_range, y_range, interval=0.5, z_value=0.0, marker_type=Marker.SPHERE, color=(1.0, 0.0, 0.0, 1.0), scale=(0.1, 0.1, 0.1)):
    """
    Create a MarkerArray with markers at every specified interval in the given x and y range.

    :param x_range: Tuple of (min_x, max_x)
    :param y_range: Tuple of (min_y, max_y)
    :param interval: Interval at which to place markers
    :param z_value: Z coordinate (default is 0.0)
    :param marker_type: Type of marker (default is Marker.SPHERE)
    :param color: Color of the markers as a tuple (r, g, b, a)
    :param scale: Scale of the markers as a tuple (x, y, z)
    :return: MarkerArray
    """
    marker_array = MarkerArray()
    marker_id = 0

    for x in np.arange(x_range[0], x_range[1] + interval, interval):
        for y in np.arange(y_range[0], y_range[1] + interval, interval):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "markers"
            marker.id = marker_id
            marker.type = marker_type
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z_value
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = scale[0]
            marker.scale.y = scale[1]
            marker.scale.z = scale[2]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]

            marker_array.markers.append(marker)
            marker_id += 1

    return marker_array

def main():
    rospy.init_node('marker_array_node')

    # Create a publisher for the marker array
    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

    # Define the range of the grid
    x_range = (-1, 6)
    y_range = (-6, 1)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        marker_array = create_marker_array(x_range, y_range, z_value=0)
        marker_pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
