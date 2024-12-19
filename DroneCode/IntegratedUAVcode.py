from __future__ import print_function
import time
from dronekit import connect, Vehicle, VehicleMode, LocationGlobalRelative
import multiprocessing
import csv
import math
from math import radians, cos, sin, sqrt, atan2, atan, tan
from SearchAlgoScript import *
from DroneSimTest import *
import time
import json
# from serial import Serial
from CameraProcess import *

# Set up option parsing to get connection string
import argparse
import logging
import cv2
import cv2.aruco as aruco
import numpy as np

logging.basicConfig(filename='flight.log',   # Name of the log file
                    level=logging.DEBUG,    # Minimum logging level (DEBUG, INFO, etc.)
                    format='%(asctime)s - %(levelname)s - %(message)s')  # Format for log message

USING_ZED_CAMERA = False  # Set to True if using the ZED camera, False otherwise
espPORT = '/dev/ttyCH341USB0'  # Change to your actual port
espBAUDRATE = 115200  # Ensure this matches the ESP32 baud rate


parser = argparse.ArgumentParser(description="Connect to a drone.")
parser.add_argument("--livedrone", action="store_true", help="Connect to a real drone instead of simulating.")
args = parser.parse_args()

# Set SIMULATE_DRONE based on the --livedrone flag
SIMULATE_DRONE = not args.livedrone  # False if --livedrone is provided, otherwise True
ALTITUDE = 4

def drone_control(location_queue):
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

def search_algorithm(marker_queue, isMarkerFound):
    while True:
        if not marker_queue.empty():
            marker_id = marker_queue.get()
            if marker_id == 0:
                isMarkerFound.value = True
            else:
                isMarkerFound.value = False

def camera_run(marker_queue, distance_to_marker_queue):
    camera = Camera(USING_ZED_CAMERA)
    frame_width = 1280
    frame_height = 720
    fps = 30
    camera_matrix, dist_coeffs = load_calibration(CALIBRATION_FILE_PATH)
    
    while True:
        frame = camera.get_frame()
        if frame is None:
            break

        camera_position, processed_frame = detect_markers(frame, marker_queue, camera_matrix, dist_coeffs, 0)

        if camera_position is not None:
            marker_id, tvec = camera_position
            distance_to_marker_queue.put(tvec)
        # Display camera positions in the corner of the window
        display_camera_position(processed_frame, camera_position)
        
        cv2.imshow("Camera Feed", processed_frame)
        
        if cv2.waitKey(1) == ord('q'):
            break
    
    camera.close()

def comms(ser, isMarkerFound, location_queue):
    while True:
        if not location_queue.empty():
            locationTuple = location_queue.get()
            data = str(locationTuple) + str(isMarkerFound.value)
            # Send the JSON string over serial
            ser.write(data.encode('utf-8'))
            print(f"Sent: {data}")
            logging.critical(f"Sent From Jetson: {data}")
            if(isMarkerFound.value):
                logging.critical(f"ArUco Marker Found At {locationTuple}")
            # time.sleep(5)  # Wait before sending the next message

if __name__ == "__main__":
    try:
        # Try to establish the serial connection
        print("Waiting for serial connection...")
        ser = Serial(espPORT, espBAUDRATE, timeout=1)
        
        # Give it some time to establish
        time.sleep(2)
        
        # Check if the serial port is open
        if ser.is_open:
            print("Serial connection established successfully.")
        else:
            print("Failed to establish serial connection.")
            ser.close()  # Ensure we close the port if it failed
            exit(1)

        marker_queue = multiprocessing.Queue()
        location_queue = multiprocessing.Queue()
        distance_to_marker_queue = multiprocessing.Queue()
        isMarkerFound = multiprocessing.Value('b', False)


        # Start the flight process
        flight_process = multiprocessing.Process(target=drone_control, args=(location_queue,))
        flight_process.start()

        # Start the camera and search algorithm processes
        camera_process = multiprocessing.Process(target=camera_run, args=(marker_queue, distance_to_marker_queue))
        camera_process.start()

        search_process = multiprocessing.Process(target=search_algorithm, args=(marker_queue, isMarkerFound))
        search_process.start()

        # Start the comms process, passing the serial connection
        comms_process = multiprocessing.Process(target=comms, args=(ser, isMarkerFound, location_queue))
        comms_process.start()



        # Wait for the processes to finish
        camera_process.join()
        search_process.join()
        comms_process.join()
        flight_process.join()

    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Close serial connection when done
        if ser.is_open:
            ser.close()
        print("Closed serial connection.")