from __future__ import print_function

# Ensures compiler can find DroneTest module from this file (DroneTest.TestSim.py)
import sys, os
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(project_root)

from dronekit import VehicleMode

# Project file imports
import scout.search_alg
import scout.drone_connect

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description="Connect to a drone.")
parser.add_argument("--livedrone", action="store_true", help="Connect to a real drone instead of simulating.")
args = parser.parse_args()


# Set SIMULATE_DRONE based on the --livedrone flag
SIMULATE_DRONE = not args.livedrone  # False if --livedrone is provided, otherwise True
ALTITUDE = 4


cornerCoordinates = [
    [1, 32.920440673828125, -96.94830322265625],
    [2, 32.920440673828125, -96.9479751586914],
    [3, 32.92019271850586, -96.9479751586914],
    [4, 32.92019271850586, -96.94831085205078]
]

enordaCopter = scout.drone_connect.connectMyCopter(SIMULATE_DRONE)

print(f"Starting Location: ({enordaCopter.location.global_relative_frame.lat}, {enordaCopter.location.global_relative_frame.lon})")
print("Heading: ", enordaCopter.heading)

# ARM DRONE
scout.drone_connect.arm_drone(enordaCopter)

# GENERATE WAYPOINTS FOR GUIDED FLIGHT PATH
waypoints, top_left_corner, top_right_corner, landing_zone_waypoint = scout.search_alg.run_path_generation(enordaCopter,0,6,8) #6 and 8 are rough numbers for testing 

# ADJUST AIRSPEED
print("Set default/target airspeed to 3")
enordaCopter.airspeed = 3

# TAKEOFF
scout.drone_connect.takeoff_drone(enordaCopter, 4)

# TAKE GUIDED FLIGHT
scout.drone_connect.flyInSearchPattern(enordaCopter, SIMULATE_DRONE)

print("Returning to Launch")
enordaCopter.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
enordaCopter.close()