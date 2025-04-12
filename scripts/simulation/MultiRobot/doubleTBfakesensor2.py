#!/usr/bin/python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import time, math
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, PoseWithCovarianceStamped

def loggingcsv(data):
    with open("/home/dryan/Desktop/SimulasiKamis/Tb3anjay2.csv", "a") as file:
        # Write headers only if the file is empty
        if file.tell() == 0:
            file.write("x, y, intensity, id\n")
        # Iterate over the list and write each item to the file
        for item in data:
            x, y, intensity, id = item
            file.write(f"{x}, {y}, {intensity:.2f}, {id}\n")
    rospy.loginfo("Print TB csv success")

def calculate_intensity(x, y, sources):
    intensities = []
    for (source_x, source_y) in sources:
        distance = math.sqrt((x - source_x) ** 2 + (y - source_y) ** 2)
        intensity = max(0, 10 - distance * 5)  # 2 meter means no intensity, r=2
        intensities.append(intensity)
    return max(intensities)

def fakesensor(x, y, id):
    marker_array_pub = rospy.Publisher('gas_sensor_array_tb3', MarkerArray, queue_size=10)
    
    global sources, datalog, then
    intensity = calculate_intensity(x, y, sources)
    
    marker_array = MarkerArray()
    
    # Create sphere marker
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "gas_sensor"
    marker.id = id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2

    # Function to change marker color
    red = min(1.0, intensity / 10.0)
    blue = 1.0 - red
    marker.color.r = red
    marker.color.g = 0.0
    marker.color.b = blue
    marker.color.a = 1.0

    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.3

    # Create text marker
    marker_text = Marker()
    marker_text.header = marker.header
    marker_text.ns = "gas_concentration_text"
    marker_text.id = id + 1000  # Ensure unique ID for text marker
    marker_text.type = Marker.TEXT_VIEW_FACING
    marker_text.action = Marker.ADD
    marker_text.pose.position.x = x
    marker_text.pose.position.y = y
    marker_text.pose.position.z = 0.5  # Offset the text above the marker
    marker_text.pose.orientation.w = 1
    marker_text.scale.z = 0.1  # Font size
    marker_text.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # White color
    marker_text.text = f"      {intensity:.2f}"

    marker_array.markers.append(marker)
    marker_array.markers.append(marker_text)

    data = [x, y, intensity, id]
    now = rospy.get_time()
    
    # Publish marker array
    if (now - then) > 1.0:
        marker_array_pub.publish(marker_array)
        datalog.append(data)
        then = now
    rospy.loginfo(f"Appended data: {data}")

def current_pose_callback(msg, robot_id):
    global current_pose
    current_pose[robot_id] = msg
    rospy.loginfo(f"Received pose for {robot_id}: {current_pose[robot_id].pose.pose.position.x}, {current_pose[robot_id].pose.pose.position.y}")

if __name__ == '__main__':
    try:
        global datalog, marker_id, then
        datalog = []
        current_pose = {'tb3_0': None, 'tb3_1': None}

        rospy.init_node('moving_marker_node', anonymous=True)
        # rospy.Subscriber('/tb3_0/amcl_pose', PoseWithCovarianceStamped, current_pose_callback, 'tb3_0')
        rospy.Subscriber('/tb3_1/amcl_pose', PoseWithCovarianceStamped, current_pose_callback, 'tb3_1')
        
        marker_id = 0
        offset = 0.5  # Adjust map offset
        sources = [(3.13 - offset, -3.18 + offset), (0.1 - offset, -3.4 + offset)]
        then = rospy.get_time()
        time.sleep(3)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            for robot_id in current_pose:
                if current_pose[robot_id] is not None:
                    x = current_pose[robot_id].pose.pose.position.x
                    y = current_pose[robot_id].pose.pose.position.y
                    calculate_intensity(x, y, sources)
                    fakesensor(x, y, marker_id)
                    marker_id += 1
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        loggingcsv(datalog)
        rospy.loginfo("Final data log completed.")
