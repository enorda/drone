from __future__ import print_function
import time
from dronekit import connect, Vehicle, VehicleMode, LocationGlobalRelative
import multiprocessing
import csv
import math
from math import radians, cos, sin, sqrt, atan2, atan, tan
from DroneCode.DroneProcess import *
import time
import json
from serial import Serial


# Set up option parsing to get connection string
import argparse
import logging
import cv2
import cv2.aruco as aruco
import numpy as np

def gv_control(location_queue):
    # Connect to the drone
    vehicle = connectMyCopter()
    print(f"Starting Location: , ({vehicle.location.global_relative_frame.lat}, {vehicle.location.global_relative_frame.lon})")
    location_queue.put([vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon])
    print("Heading: ", vehicle.heading)

    logging.info("Arming Drone")
    arm_drone(vehicle)

    # Outputs waypoints to csv
    waypoints, top_left_corner, top_right_corner, landing_zone_waypoint = run_path_generation(vehicle,vehicle.heading,6,8) #6 and 8 are rough numbers for testing 

    print("Set default/target airspeed to 3")
    vehicle.airspeed = 3

    logging.critical("UAV START: TAKING OFF")
    print("Set default/target airspeed to 3")
    takeoff_drone(vehicle, 4)

    # Consumes waypoints in csv, and goes to those locations
    flyInSearchPattern(vehicle, location_queue)
    
    print("Returning to Launch")
    vehicle.mode = VehicleMode("RTL")

    print("Close vehicle object")
    vehicle.close()
    logging.critical("UAV END: LANDING")