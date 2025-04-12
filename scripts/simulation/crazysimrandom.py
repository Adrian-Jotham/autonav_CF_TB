#!/usr/bin/python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import time, math
from std_msgs.msg import ColorRGBA
import random

def loggingcsv(data):
    with open("/home/dryan/Desktop/TigaRobot/CF1.csv", "w") as file:
        # Write headers
        file.write("x, y, intensity, id\n")
        # Iterate over the list and write each item to the file
        for item in data:
            x, y, intensity, id = item
            file.write(f"{x}, {y}, {intensity:.2f}, {id}\n")
    rospy.loginfo("Print CF1 csv success")
    
def calculate_intensity(x, y, sources):
    intensities = []
    for (source_x, source_y) in sources:
        distance = math.sqrt((x - source_x) ** 2 + (y - source_y) ** 2)
        intensity = max(0, 10 - distance * 5)  # 2 meter means no intensity, r=2
        intensities.append(intensity)
    return max(intensities)

def fakesensor(x, y, id):
    marker_array_pub = rospy.Publisher('gas_sensor_array_cf_1', MarkerArray, queue_size=10)
    
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
    marker.pose.position.z = 0.8

    # Create text marker
    marker_text = Marker()
    marker_text.header = marker.header
    marker_text.ns = "gas_concentration_text"
    marker_text.id = id + 1000  # Ensure unique ID for text marker
    marker_text.type = Marker.TEXT_VIEW_FACING
    marker_text.action = Marker.ADD
    marker_text.pose.position.x = x
    marker_text.pose.position.y = y
    marker_text.pose.position.z = 1.0  # Offset the text above the marker
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

def create_moving_marker(x_start, y_start, x_end, y_end, velocity=0.35):
    """
    Create a moving marker that navigates from start to end coordinates.

    :param x_start: Starting x coordinate
    :param y_start: Starting y coordinate
    :param x_end: Ending x coordinate
    :param y_end: Ending y coordinate
    :param velocity: Velocity of the marker
    """
    global last_x, last_y, marker_id
    offset = 0.5
    x_end -= offset
    y_end += offset
    marker_pub = rospy.Publisher('Crazyflie_1', Marker, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz update rate

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "moving_marker"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # Calculate direction vector and normalize it
    direction = np.array([x_end - x_start, y_end - y_start], dtype=float)
    distance = np.linalg.norm(direction)
    
    if distance != 0:  # Avoid division by zero
        direction = direction / distance

    current_position = np.array([x_start, y_start], dtype=float)
    target_position = np.array([x_end, y_end], dtype=float)

    while not rospy.is_shutdown():
        # Update marker position
        marker.pose.position.x = current_position[0]
        marker.pose.position.y = current_position[1]
        marker.pose.position.z = 0.8

        # Publish marker
        marker_pub.publish(marker)

        # Move the marker towards the target position
        current_position += direction * velocity / 10.0

        fakesensor(current_position[0], current_position[1], marker_id)
        marker_id += 1

        # Stop if the marker has reached or passed the target position
        if np.linalg.norm(current_position - target_position) <= velocity / 10.0:
            last_x = x_end
            last_y = y_end
            break

        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('moving_marker_node', anonymous=True)

        global last_x, last_y, datalog, marker_id, then
        datalog = []
        marker_id = 0
        offset = 0.5  # Adjust map offset

        sources = [(3.13 - offset, -3.18 + offset), (0.1 - offset, -3.4 + offset)]
        then = rospy.get_time()
        time.sleep(3)

        coordinates = [
            (0, 0), (0, -1), (0, -2), (0, -3), 
            (1, 0), (1, -1), (1, -2), (1, -3), (1, -3.5), 
            (2, 0), (2, -1), (2, -2), (2, -3), (2, -4),  
            (3, 0), (3, -1), (3, -3.4), (3, -4), (3, -5), 
            (4, 0), (4, -1), (4, -2), (4, -3), (4, -4),
            (5,0), (5,-1),
        ]

        last_x, last_y = coordinates[0]
        visited_coords = set()
        visited_coords.add((last_x, last_y))

        while not rospy.is_shutdown():
            next_coords = [coord for coord in coordinates if coord not in visited_coords]
            if not next_coords:
                rospy.loginfo("All waypoints visited. Exiting.")
                break
            next_coord = random.choice(next_coords)
            create_moving_marker(last_x, last_y, next_coord[0], next_coord[1])
            last_x, last_y = next_coord
            visited_coords.add(next_coord)

        # loggingcsv(datalog)
    except rospy.ROSInterruptException:
        # time.sleep(1)
        loggingcsv(datalog)
