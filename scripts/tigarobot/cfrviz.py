#!/usr/bin/python3

import time
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import geometry_msgs.msg
from tf2_msgs.msg import TFMessage

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
import csv
import os

# Function to log data to CSV
def loggingcsv(data, cf_number):
    waktu_sekarang = time.localtime()
    tanggal = time.strftime("%d-%m-%Y", waktu_sekarang)
    hari = time.strftime("%A", waktu_sekarang)
    nama_file = f'cf{cf_number}_{hari}_{tanggal}.csv'
    
    with open(f"/home/dryan/Desktop/TigaRobot/{nama_file}", "w", newline='') as file:
        writer = csv.writer(file)
        # writer.writerow(['stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z', 
        #                  'kalman.q0', 'kalman.q1', 'kalman.q2', 'kalman.q3', 
        #                  'TGS8100.intensity', 'seconds', 'nseconds'])
        writer.writerow(['stateEstimate.x', 'stateEstimate.y',  
                         'TGS8100.intensity', 'seconds', 'nseconds'])
        for item in data:
            row = [item['stateEstimate.x'], item['stateEstimate.y'],
                   item['TGS8100.intensity'], item['seconds'], item['nseconds']]
            writer.writerow(row)
    print("Selesai")

def get_color_for_concentration(concentration):
    if concentration >= 2.5:
        return ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red for high concentration
    elif concentration >= 1.4:
        return ColorRGBA(1.0, 1.0, 0.0, 1.0)  # Yellow for medium concentration
    else:
        return ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green for low concentration

def publish_gas(data, id, pub_gas, datalog):
    global then
    now = rospy.get_time()
    marker_array = MarkerArray()
    marker = Marker()
    marker.header.frame_id = "tb3/map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "gas_concentration_cf"
    marker.id = id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.08  # Marker diameter
    marker.scale.y = 0.08
    marker.scale.z = 0.08
    marker.color.a = 1.0  # Alpha

    # Set marker color based on gas concentration
    concentration = data['TGS8100.intensity']
    marker.color = get_color_for_concentration(concentration)

    # Set marker position
    point = Point()
    point.x = data['stateEstimate.x']
    point.y = data['stateEstimate.y']
    point.z = data['stateEstimate.z']
    marker.pose.position = point

    # Add text label
    marker_text = Marker()
    marker_text.header = marker.header
    marker_text.ns = "gas_concentration_text"
    marker_text.id = id
    marker_text.type = Marker.TEXT_VIEW_FACING
    marker_text.action = Marker.ADD
    marker_text.pose.position = point
    marker_text.pose.position.z += 0  # Offset the text above the marker
    marker_text.pose.orientation.w = 1
    marker_text.scale.z = 0.1  # Font size
    marker_text.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # White color
    marker_text.text = f"      {concentration:.2f}"  # Format concentration value

    marker_array.markers.append(marker)
    marker_array.markers.append(marker_text)

    if (now - then) > 1.0:
        pub_gas.publish(marker_array)
        datalog.append(data)
        then = now
    
def publish_tf(data, pub_tf):
    transform = geometry_msgs.msg.TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "tb3/map"  # Change this to your desired parent frame
    transform.child_frame_id = "crazyflie/base_link"  # Change this to your desired child frame

    transform.transform.translation.x = data['stateEstimate.x']  
    transform.transform.translation.y = data['stateEstimate.y']
    transform.transform.translation.z = data['stateEstimate.z']

    transform.transform.rotation.x = data['kalman.q1']
    transform.transform.rotation.y = data['kalman.q2']
    transform.transform.rotation.z = data['kalman.q3']
    transform.transform.rotation.w = data['kalman.q0']

    tf_msg = TFMessage([transform])
    pub_tf.publish(tf_msg)

# Callback function to handle received log data
def log_data_callback(timestamp, data, logconf, cf_number, pub_tf, pub_gas, datalog):
    data['seconds'] = rospy.get_rostime().secs
    data['nseconds'] = rospy.get_rostime().nsecs

    publish_tf(data, pub_tf)
    publish_gas(data, timestamp, pub_gas, datalog)

# Function to activate LED bit mask for visual confirmation
def activate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 255)

# Function to deactivate LED bit mask
def deactivate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 0)

# Function to check the LEDs
def light_check(scf):
    activate_led_bit_mask(scf)
    time.sleep(2)
    deactivate_led_bit_mask(scf)
    time.sleep(2)

# Function to make the Crazyflie take off to a specific height
def take_off(scf, height):
    commander = scf.cf.high_level_commander
    commander.takeoff(height, 2.0)
    time.sleep(3)

# Function to make the Crazyflie land
def land(scf):
    commander = scf.cf.high_level_commander
    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()

# Function to perform the hover sequence: take off and land
def hover_sequence(scf, height):
    take_off(scf, height)
    run_square_sequence(scf)
    land(scf)

def run_square_sequence(scf):
    box_size = 1
    flight_time = 2

    commander = scf.cf.high_level_commander

    commander.go_to(box_size, 0, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(0, box_size, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(-box_size, 0, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

    commander.go_to(0, -box_size, 0, 0, flight_time, relative=True)
    time.sleep(flight_time)

# Wrapper function to pass the height argument correctly
def hover_sequence_wrapper(scf, height):
    hover_sequence(scf, height)

# Function to log data
def log_data(scf, cf_number, pub_tf, pub_gas, datalog):
    log_conf = LogConfig(name='MyLog', period_in_ms=10)
    log_conf.add_variable('stateEstimate.x', 'float')
    log_conf.add_variable('stateEstimate.y', 'float')
    log_conf.add_variable('stateEstimate.z', 'float')
    log_conf.add_variable('kalman.q0', 'FP16')
    log_conf.add_variable('kalman.q1', 'FP16')
    log_conf.add_variable('kalman.q2', 'FP16')
    log_conf.add_variable('kalman.q3', 'FP16')
    log_conf.add_variable('TGS8100.intensity', 'FP16')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(lambda timestamp, data, logconf: log_data_callback(timestamp, data, logconf, cf_number, pub_tf, pub_gas, datalog))
    log_conf.start()

# List of Crazyflie URIs and corresponding heights
uris = {
    'radio://0/80/2M/E7E7E7E7E7': 1.0,
    'radio://0/60/2M/E7E7E7E701': 1.0,
    # Add more URIs and heights if you want more copters in the swarm
}

if __name__ == '__main__':
    # Initialize the Crazyflie API
    cflib.crtp.init_drivers()

    # Initialize ROS node
    rospy.init_node('crazyflie_swarm', anonymous=True)
    namespace = rospy.get_namespace()
    pub_tf = rospy.Publisher('/tf', TFMessage, queue_size=100)
    pub_gas = rospy.Publisher(f'{namespace}gas_concentration_markers', MarkerArray, queue_size=100)

    then = rospy.get_time()

    # Create a factory for Crazyflie instances
    factory = CachedCfFactory(rw_cache='./cache')

    # Create the swarm with the provided URIs and factory
    with Swarm(uris.keys(), factory=factory) as swarm:
        print('Connected to Crazyflies')

        # Perform LED check in parallel
        swarm.parallel_safe(light_check)

        # Prepare arguments for hover_sequence_wrapper
        args_dict = {uri: [height] for uri, height in uris.items()}

        # Assign Crazyflie numbers for file naming and datalogs
        cf_numbers = {uri: idx + 1 for idx, uri in enumerate(uris.keys())}
        datalogs = {uri: [] for uri in uris.keys()}

        # Start logging data in parallel
        swarm.parallel_safe(lambda scf: log_data(scf, cf_numbers[scf.cf.link_uri], pub_tf, pub_gas, datalogs[scf.cf.link_uri]))
        time.sleep(10)

        # Perform the hover sequence in parallel, each drone with its specific height
        swarm.parallel_safe(hover_sequence_wrapper, args_dict=args_dict)

    # Log data to CSV after swarm flight
    for uri in uris.keys():
        loggingcsv(datalogs[uri], cf_numbers[uri])
