#!/usr/bin/python3

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.position_hl_commander import PositionHlCommander
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
import time
import csv, os

def loggingcsv(data):
    waktu_sekarang = time.localtime()
    tanggal = time.strftime("%d-%m-%Y", waktu_sekarang)
    hari = time.strftime("%A", waktu_sekarang)
    nama_file = str(hari + '-' + tanggal + '.csv')
    
    kolom = ['stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z', 
             'kalman.q0', 'kalman.q1', 'kalman.q2', 'kalman.q3', 
             'TGS8100.intensity']
    
    # datalist = [data.get(key, '') for key in kolom]

    nama_file = 'data.csv'
    with open("/home/dryan/Desktop/logterbang.csv", "w") as file:
    # Iterate over the list and write each item to the file
        for item in data:
            for baris in item:
                awal = list(baris.values())
                konversi_list = [str(item) for item in awal]
                konversi = ', '.join(konversi_list)
                file.write(konversi+ "\n")

def get_color_for_concentration(concentration):
    if concentration >= 2.5:
        return ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red for high concentration
    elif concentration >= 1.4:
        return ColorRGBA(1.0, 1.0, 0.0, 1.0)  # Yellow for medium concentration
    else:
        return ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green for low concentration

def publish_gas(data, id):
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
    # marker_text.text = f"Concentration: {concentration:.2f}"  # Format concentration value
    marker_text.text = f"      {concentration:.2f}"  # Format concentration value

    marker_array.markers.append(marker)
    marker_array.markers.append(marker_text)

    if (now - then) > 1.0:
        pub_gas.publish(marker_array)
        then = now
    
def publish_tf(data):
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
    # print(data)
    # print(tf_msg) #debugging option
    pub_tf.publish(tf_msg)

# Callback function to handle received log data
def log_data_callback(timestamp, data, logconf):
    # print('[{}] {}'.format(timestamp, data))
    data['seconds']=rospy.get_rostime().secs
    data['nseconds']=rospy.get_rostime().nsecs
    #nanti ubah di loginfo untuk log 
    # rospy.loginfo('[{}] {}'.format(timestamp, data))
    # rospy.loginfo(data)

    global datalog
    datalog.append([data])

    publish_tf(data)
    publish_gas(data, timestamp)
    

if __name__ == "__main__":
    # Initialize the Crazyflie API
    cflib.crtp.init_drivers()

    # logging.basicConfig(level=logging.ERROR)
    rospy.init_node('crazyflie', anonymous=True)
    namespace = rospy.get_namespace()
    # pub_tf = rospy.Publisher(f'{namespace}tf', TFMessage, queue_size=100)
    pub_gas = rospy.Publisher(f'{namespace}gas_concentration_markers', MarkerArray, queue_size=100)
    pub_tf = rospy.Publisher('/tf', TFMessage, queue_size=100)
    # pub_gas = rospy.Publisher('/gas_concentration_markers', MarkerArray, queue_size=100)


    # URI to connect to the Crazyflie (change this to match your Crazyflie's URI)
    uri = 'radio://0/90/2M/E7E7E7E7E7'
    # uri = rospy.get_param('uri','radio://0/100/2M/E7E7E7E7E7')

    # Define the log configuration
    log_conf = LogConfig(name='MyLog', period_in_ms=10)
    log_conf.add_variable('stateEstimate.x', 'float')
    log_conf.add_variable('stateEstimate.y', 'float')
    log_conf.add_variable('stateEstimate.z', 'float')
    log_conf.add_variable('kalman.q0', 'FP16')
    log_conf.add_variable('kalman.q1', 'FP16')
    log_conf.add_variable('kalman.q2', 'FP16')
    log_conf.add_variable('kalman.q3', 'FP16')
    log_conf.add_variable('TGS8100.intensity', 'FP16')

    then =rospy.get_time()
    datalog = []
    # Connect to the Crazyflie
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(log_data_callback)
        log_conf.start()
        rospy.loginfo("start logging")

        # time.sleep(3.0)
        # r = rospy.Rate(10) # 10hz, for publishing or just waiting
        # while not rospy.is_shutdown():
        #     #might publish something
        #     r.sleep()

        # Create a PositionCommander object for flying
        # with PositionHlCommander(
        #                 cf,
        #                 x=2.0, y=-3.0, z=0.0,
        #                 default_velocity=0.35,
        #                 default_height=0.8,
        #                 controller=PositionHlCommander.CONTROLLER_PID) as pc:

        #     # Take off to a height of 0.5 meters
        #     # pc.take_off(0.5)

        #     # Go to a specific position
        #     # pc.go_to(3.0, 2.0)
        #     # time.sleep(3)

        #     #Terbang WP baru
        #     # pc.move_distance(0, 2.0, 0, velocity=None)
        #     # pc.move_distance(1.0, 0 , 0, velocity=None)

        #     # pc.move_distance(0, -3.0, 0, velocity=None)
        #     # pc.move_distance(1.0, 0 , 0, velocity=None)

        #     # pc.move_distance(0, 3.0, 0, velocity=None)
        #     # pc.move_distance(1.0, 0 , 0, velocity=None)

        #     # pc.move_distance(0, -4.0 , 0, velocity=None)

        #     #Terbang skenario statis
        #     time.sleep(3)
        #     pc.move_distance(1, 0 , 0, velocity=None)
        #     time.sleep(3)
        #     pc.move_distance(-1, 0 , 0, velocity=None)

        #     # pc.go_to(1,0, -3,0)
        #     # pc.go_to(1,0, -2,0)
        #     # pc.go_to(1,0, -1,0, velocity=0.35)

        #     # pc.go_to(2,0, -1,0, velocity=0.35)
        #     # pc.go_to(2,0, -2,0)
        #     # pc.go_to(2,0, -3,0)
        #     # pc.go_to(2,0, -4,0, velocity=0.35)

        #     # pc.go_to(3,0, -4,0, velocity=0.35)
        #     # pc.go_to(3,0, -3,0)
        #     # pc.go_to(3,0, -2,0)
        #     # pc.go_to(3,0, -1,0, velocity=0.35)

        #     # pc.go_to(4,0, -1,0)
        #     # pc.go_to(4,0, -2,0)
        #     # pc.go_to(4,0, -3,0)
        #     # pc.go_to(4,0, -4,0)
        #     # pc.go_to(4,0, -5,0)


        #     # pc.go_to(1.11, 1.06)
        #     # pc.go_to(3.38, 1.0)
        #     # pc.go_to(4.51, 4.2) #crash
        #     # pc.go_to(4.31, 4.0) #aman
        #     # pc.go_to(2.0, 3.3)
        #     # pc.go_to(1.11, 1.06)

        #     # Land the Crazyflie
        #     pc.land()

        # rospy.loginfo("The end of logging")
        # Stop logging
        log_conf.stop()
        loggingcsv(datalog)
        # Disconnect from the Crazyflie
        cf.close_link()
