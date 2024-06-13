#!/usr/bin/python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
import math

def create_marker(x, y, intensity, marker_id):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.1  # Slightly above the ground
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    # Calculate color based on intensity
    red = min(1.0, intensity / 10.0)
    blue = 1.0 - red
    marker.color.r = red
    marker.color.g = 0.0
    marker.color.b = blue
    marker.color.a = 1.0

    marker.id = marker_id

    return marker

def calculate_intensity(x, y, sources):
    intensities = []
    for (source_x, source_y) in sources:
        distance = math.sqrt((x - source_x) ** 2 + (y - source_y) ** 2)
        intensity = max(0.1, 10 - distance * 5)
        intensities.append(intensity)
    return max(intensities)

def main():
    rospy.init_node('gas_source_marker_array')

    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    markers = MarkerArray()
    marker_id = 0
    offset = 0.5 #atur jarak map 0.0 (0.0 = tb) ke actual 0.0
    sources = [(3.13 - offset, -3.18 + offset), (0.1 - offset, -3.4 + offset)]

    for x in range(-4, 25):
        for y in range(-24, 3):
            real_x = x * 0.25
            real_y = y * 0.25
            intensity = calculate_intensity(real_x, real_y, sources)
            marker = create_marker(real_x, real_y, intensity, marker_id)
            markers.markers.append(marker)
            marker_id += 1

    while not rospy.is_shutdown():
        marker_pub.publish(markers)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
