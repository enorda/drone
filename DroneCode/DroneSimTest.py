from __future__ import print_function
import time
from dronekit import connect, Vehicle, VehicleMode, LocationGlobalRelative

import csv
import math
from math import radians, cos, sin, sqrt, atan2, atan, tan
from SearchAlgoScript import *

import multiprocessing
# Set up option parsing to get connection string
import argparse

parser = argparse.ArgumentParser(description="Connect to a drone.")
parser.add_argument("--livedrone", action="store_true", help="Connect to a real drone instead of simulating.")
args = parser.parse_args()


# Set SIMULATE_DRONE based on the --livedrone flag
SIMULATE_DRONE = not args.livedrone  # False if --livedrone is provided, otherwise True
ALTITUDE = 4

# Used to connect to copter with args from command line
def connectMyCopter():
    if SIMULATE_DRONE:
        # Create a SITL drone instance instead of launching one beforehand
        import dronekit_sitl
        sitl = dronekit_sitl.start_default(32.92019271850586, -96.94831085205078)
        connection_string = sitl.connection_string()
        vehicle = connect(connection_string, wait_ready=True)

    else:
        vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True) 
        '''
        This is the connect they were using in 23-24 pqqtest2
        FIX THIS FOR FLIGHT NOT SURE: https://dronekit.netlify.app/guide/connecting_vehicle.html
        '''

    return vehicle

# Used to arm the drone
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
    while not vehicle.armed:  # While the vehicle has not been armed
        print("Waiting for drone to arm: ", {vehicle.system_status.state})
        time.sleep(1)  # Wait one second before checking if drone has been armed
    print("The drone has now been armed")

    # Check if GPS is functioning
    while vehicle.gps_0.fix_type < 2:  # Ensure GPS is ready
        print(" Waiting for GPS to initialise...", vehicle.gps_0.fix_type)
        time.sleep(1)
    print("Copter GPS Ready")

# Used to take off the drone to a specific altitude
def takeoff_drone(vehicle, targetAltitude):
    print("Taking off!")
    vehicle.simple_takeoff(targetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    # after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= targetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def getCurrentLocation(vehicle):
    currentLoc = (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
    return currentLoc
 
def flyInSearchPattern(vehicle, location_queue):
    search_waypoints = load_waypoints_from_csv('generated_search_pattern_waypoints.csv')
    # Iterate over waypoints, expecting lists of [latitude, longitude]
    if SIMULATE_DRONE:
        for wp in search_waypoints:
            currentWP = (wp.lat, wp.lon)
            print("Waypoint:", currentWP)
            # Go to the waypoint
            vehicle.simple_goto(wp)
            #time.sleep(20)
            while(equirectangular_approximation(getCurrentLocation(vehicle),currentWP) > .5):
                location_queue.put([vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon])
                print(f"Current Location: , ({vehicle.location.global_relative_frame.lat}, {vehicle.location.global_relative_frame.lon})")
                print("Distance to WP:", equirectangular_approximation(getCurrentLocation(vehicle),currentWP))
                time.sleep(1)
    else:
        for wp in search_waypoints:
            currentWP = (wp.lat, wp.lon)
            print("Waypoint:", currentWP)
            # Go to the waypoint
            vehicle.simple_goto(wp)
            #time.sleep(20)
            while(equirectangular_approximation(getCurrentLocation(vehicle),currentWP) > .5): 
                print(vehicle.location.global_relative_frame.alt)
                location_queue.put([vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon])
                print(f"Current Location: , ({vehicle.location.global_relative_frame.lat}, {vehicle.location.global_relative_frame.lon})")
                print("Distance to WP:", equirectangular_approximation(getCurrentLocation(vehicle),currentWP))
                time.sleep(1)

# if __name__ == "__main__":
#     try:
        
#         flight_process = multiprocessing.Process(target=drone_control) #, args=(location_queue, )
#         flight_process.start()

#         # Wait for the processes to finish

#         flight_process.join()

#     except Exception as e:
#         print(f"Error: {e}")
#     finally:

#         print("Closed serial connection.")
