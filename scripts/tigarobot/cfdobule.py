import time
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from cflib.crazyflie.log import LogConfig

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
    # time.sleep(10)
    run_square_sequence(scf)
    land(scf)

def run_square_sequence(scf):
    box_size = 1
    flight_time = 2

    commander= scf.cf.high_level_commander

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
def log_data(scf):
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
    log_conf.data_received_cb.add_callback(lambda timestamp, data, logconf: log_data_callback(timestamp, data, logconf, scf.cf.link_uri))
    log_conf.start()

# Callback function to handle the received log data
def log_data_callback(timestamp, data, logconf, uri):
    data_dict = {key: value for key, value in data.items()}
    print(f'[{timestamp}] {uri}: {data_dict}')

# List of Crazyflie URIs and corresponding heights
uris = {
    'radio://0/80/2M/E7E7E7E7E7': 1.0,
    'radio://0/60/2M/E7E7E7E701': 1.5,
    # Add more URIs and heights if you want more copters in the swarm
}

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    # Create a factory for Crazyflie instances
    factory = CachedCfFactory(rw_cache='./cache')

    # Create the swarm with the provided URIs and factory
    with Swarm(uris.keys(), factory=factory) as swarm:
        print('Connected to Crazyflies')

        # Perform LED check in parallel
        swarm.parallel_safe(light_check)

        # Prepare arguments for hover_sequence_wrapper
        args_dict = {uri: [height] for uri, height in uris.items()}

        # Start logging data in parallel
        swarm.parallel_safe(log_data)
        time.sleep(5)

        # Perform the hover sequence in parallel, each drone with its specific height
        swarm.parallel_safe(hover_sequence_wrapper, args_dict=args_dict)
