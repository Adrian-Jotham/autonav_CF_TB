#!/usr/bin/python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
import pandas as pd
import numpy as np

def create_marker_array_from_excel(file_path, marker_type=Marker.SPHERE, scale=(0.1, 0.1, 0.1)):
    """
    Create a MarkerArray based on x, y coordinates and concentration values from an Excel file.

    :param file_path: Path to the Excel file
    :param marker_type: Type of marker (default is Marker.SPHERE)
    :param scale: Scale of the markers as a tuple (x, y, z)
    :return: MarkerArray
    """
    df = pd.read_excel(file_path, engine='openpyxl')

    marker_array = MarkerArray()

    for index, row in df.iterrows():
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "markers"
        marker.id = index
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = row['x']
        marker.pose.position.y = row['y']
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.r = 1.0  # Red color
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = row['concentration']  # Assuming concentration is normalized between 0 and 1

        marker_array.markers.append(marker)

    return marker_array

def main():
    rospy.init_node('marker_array_from_excel_node')

    # Create a publisher for the marker array
    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

    # Path to the Excel file
    file_path = '/home/dryan/catkin_ws/src/auto_nav/scripts/gas_concentration_grid.xlsx'

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        marker_array = create_marker_array_from_excel(file_path)
        marker_pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
