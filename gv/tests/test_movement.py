from __future__ import print_function
import time
import dronekit_sitl
from dronekit import connect, VehicleMode, LocationGlobalRelative
from collections.abc import MutableMapping  # Fix for collections error

# Logger Setup
import logging
AVC_LOG = 25
logging.addLevelName(AVC_LOG, "AVC")
def log_avc(self, message, *args, **kwargs):
    if self.isEnabledFor(AVC_LOG):
        self._log(AVC_LOG, message, args, **kwargs)
logging.Logger.avc = log_avc
class CustomLevelFilter(logging.Filter):
    def filter(self, record):
        return record.levelno == AVC_LOG
logging.basicConfig(
    filename='flight.log',
    level=AVC_LOG,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("FlightLogger")

import argparse
parser = argparse.ArgumentParser(description="Connect to a drone.")
parser.add_argument("--livedrone", action="store_true", help="Connect to a real drone instead of simulating.")
args = parser.parse_args()
SIMULATE_DRONE = not args.livedrone

def connectMyCopter():
    if SIMULATE_DRONE:
        # Create a SITL drone instance instead of launching one beforehand
        sitl = dronekit_sitl.start_default(32.92019271850586, -96.9479751586914)
        connection_string = sitl.connection_string()
        vehicle = connect(connection_string, wait_ready=True)

    else:
        vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True) 
        '''
        This is the connect they were using in 23-24 pqqtest2
        FIX THIS FOR FLIGHT NOT SURE: https://dronekit.netlify.app/guide/connecting_vehicle.html
        '''

    return vehicle

def arm_drone(vehicle):
    while not vehicle.is_armable:  # While the drone hasn't been armed
        print("Waiting for drone to become armable")
        time.sleep(1)  # Wait one second before checking if drone is armable
    print("The drone is now armable")

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != 'GUIDED':  # While drone is not in guided mode
        print("The drone is not in guided mode yet")
        time.sleep(1)  # Wait one second before checking if drone is in guided mode
    print("The drone is now in guided mode")

    vehicle.armed = True
   # while not vehicle.armed:  # While the vehicle has not been armed
        #print("Waiting for drone to arm")
        #time.sleep(1)  # Wait one second before checking if drone has been armed
    print("The drone has now been armed")

    # Check if GPS is functioning
    while vehicle.gps_0.fix_type < 2:  # Ensure GPS is ready
        print(" Waiting for GPS to initialise...", vehicle.gps_0.fix_type)
        time.sleep(1)
    print("Copter GPS Ready")

copter = connectMyCopter()
arm_drone(copter)
print(f"Moving forward meter(s)")
current_location = copter.location.global_relative_frame
target_location = LocationGlobalRelative(current_location.lat + (32/111111), current_location.lon, current_location.alt)
copter.simple_goto(target_location)
time.sleep(5)  # Adjust for movement timing
print("Movement complete")