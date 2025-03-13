from __future__ import print_function
import time
from dronekit import connect, Vehicle, VehicleMode, LocationGlobalRelative
import multiprocessing
import csv
import math
from math import radians, cos, sin, sqrt, atan2, atan, tan
from SearchAlgoScript import *
from DroneProcess import *
import time
import json
from serial import Serial
from CameraProcess import *

# Set up option parsing to get connection string
import argparse
import logging
import cv2
import cv2.aruco as aruco
import numpy as np
#Added this to save video image
import os
from datetime import datetime

"""
#LOGGER SETUP TO USE CUSTOM LOGS REQUIRED PER AVC RULEBOOK
AVC_LOG = 25  # Pick a value that does not clash with existing levels
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
    level=AVC_LOG,  # Set to the custom level
    format='%(asctime)s - %(levelname)s - %(message)s'
)
root_logger = logging.getLogger()
for handler in root_logger.handlers:
    handler.addFilter(CustomLevelFilter())
logger = logging.getLogger("FlightLogger")

# Set up logging to log both to flight.log and a text file

# Set up a StreamHandler to log to a text file
text_file_handler = logging.FileHandler('flight_output.txt', mode='w')  # 'w' to overwrite the file every time
text_file_handler.setLevel(AVC_LOG)  # Set to the custom level

# Add formatter for the text file handler
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
text_file_handler.setFormatter(formatter)

# Add the text file handler to the logger
root_logger.addHandler(text_file_handler)
"""

log_folder = "DEL-Logging Folder"
# Ensure the log folder exists
if not os.path.exists(log_folder):
    os.makedirs(log_folder)
text_folder = "DEL-Text Folder"
if not os.path.exists(text_folder):
    os.makedirs(text_folder)
record_folder = "DEL-Recording Folder"
if not os.path.exists(record_folder):
    os.makedirs(record_folder)
# Custom log level setup for AVC
AVC_LOG = 25  # Pick a value that does not clash with existing levels
logging.addLevelName(AVC_LOG, "AVC")

def log_avc(self, message, *args, **kwargs):
    if self.isEnabledFor(AVC_LOG):
        self._log(AVC_LOG, message, args, **kwargs)

logging.Logger.avc = log_avc

# Custom level filter
class CustomLevelFilter(logging.Filter):
    def filter(self, record):
        return record.levelno == AVC_LOG

# Function to generate a unique filename
def get_unique_filename(base_filename):
    """
    This function checks if the file already exists.
    If it does, it appends a number to the filename to make it unique.
    """
    file_name, file_extension = os.path.splitext(base_filename)
    counter = 0
    new_filename = base_filename
    while os.path.exists(new_filename):
        new_filename = f"{file_name}_{counter}{file_extension}"
        counter += 1
    return new_filename

current_datetime = datetime.now().strftime("%m-%d-%y %I:%M")
log_filename = get_unique_filename(os.path.join(log_folder,f"{current_datetime} flight.log"))
# Set up logging to flight.log
logging.basicConfig(
    filename=log_filename,  # Log file for logger.avc
    level=AVC_LOG,  # Set to the custom level
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# Add a custom filter to the root logger
root_logger = logging.getLogger()
for handler in root_logger.handlers:
    handler.addFilter(CustomLevelFilter())

# Create a logger for FlightLogger
logger = logging.getLogger("FlightLogger")

# Custom stream class to write to both terminal and file
class Output:
    def __init__(self, filename, mode="w"):
        self.terminal = sys.stdout
        self.file = open(filename, mode)

    def write(self, message):
        self.terminal.write(message)  # Write to terminal
        self.file.write(message)  # Write to file

    def flush(self):
        self.terminal.flush()
        self.file.flush()

# Check and get unique filename for the output text file
unique_output_filename = get_unique_filename(os.path.join(text_folder, f"{current_datetime} flight_output.txt"))

# Redirect sys.stdout to the custom Output class
sys.stdout = Output(unique_output_filename)
sys.stderr = sys.stdout

USING_ZED_CAMERA = True  # Set to True if using the ZED camera, False otherwise
espPORT = '/dev/ttyUSB0'  # Change to your actual port
espBAUDRATE = 115200  # Ensure this matches the ESP32 baud rate
frame_width = 1280
frame_height = 720
#Added this to grab frames for video
fps = 30
#end

parser = argparse.ArgumentParser(description="Connect to a drone.")
parser.add_argument("--livedrone", action="store_true", help="Connect to a real drone instead of simulating.")
args = parser.parse_args()

# Set SIMULATE_DRONE based on the --livedrone flag
SIMULATE_DRONE = not args.livedrone  # False if --livedrone is provided, otherwise True
ALTITUDE = 4


def drone_control(location_queue, isMarkerFound, distance_to_marker_queue):
    # Connect to the drone
    vehicle = connectMyCopter()
    #Wait for seach algorithm to start TN 2/28
    search_ready.wait()
    print("Search ready")
    camera_matrix, dist_coeffs = load_calibration(CALIBRATION_FILE_PATH)
    if(USING_ZED_CAMERA):
        ground_coverage_width, ground_coverage_height = get_image_dimensions_meters((frame_width,frame_height), camera_matrix,
                                                                                                ALTITUDE)
    else:
        ground_coverage_width, ground_coverage_height = 5,3
    print(f"W:{ground_coverage_width}H:{ground_coverage_height}")

    print(f"Starting Location: , ({vehicle.location.global_relative_frame.lat}, {vehicle.location.global_relative_frame.lon})")
    location_queue.put([vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon])
    print("Heading: ", vehicle.heading)

    logger.avc("Arming Drone")
    arm_drone(vehicle)

    # Outputs waypoints to csv
    waypoints, top_left_corner, top_right_corner, landing_zone_waypoint = run_path_generation(vehicle,vehicle.heading,ground_coverage_width,ground_coverage_height) #6 and 8 are rough numbers for testing 

    print("Set default/target airspeed to 3")
    vehicle.airspeed = 3

    logger.avc("UAV START: TAKING OFF")
    print("Set default/target airspeed to 3")
    takeoff_drone(vehicle, ALTITUDE)

    # Consumes waypoints in csv, and goes to those locations
    flyInSearchPattern(vehicle, location_queue, isMarkerFound, distance_to_marker_queue)
    
    print("Returning to Launch")
    vehicle.mode = VehicleMode("RTL")

    print("Close vehicle object")
    vehicle.close()
    logger.avc("UAV END: LANDING")

def search_algorithm(marker_queue, isMarkerFound):
    #Wait for the comms to be ready TN 2/28
    comms_ready.wait()
    print("Comms Ready")
    #Set search queue to ready TN 2/28
    search_ready.set()
    while True:
        if not marker_queue.empty():
            marker_id = marker_queue.get()
            if marker_id == 0:
                with isMarkerFound.get_lock():
                    isMarkerFound.value = True
            else:
                with isMarkerFound.get_lock():
                    isMarkerFound.value = False


def camera_run(marker_queue, distance_to_marker_queue):
   #Tell camera Queue is ready TN 2/28
    camera_ready.set()
    print("Camera ready")
    camera = Camera(USING_ZED_CAMERA, frame_width, frame_height)
    camera_matrix, dist_coeffs = load_calibration(CALIBRATION_FILE_PATH)
    cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL) #Added this to make the windows adjustable but havent tested
    
    # Initialize VideoWriter to save video
    output_filename = os.path.join(record_folder, f"{current_datetime} Record_while_flying.avi")
    unique_output_filename = get_unique_filename(output_filename)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Codec for AVI file
    video_writer = cv2.VideoWriter(unique_output_filename , fourcc, fps, (frame_width, frame_height))
    print(f"Saving video to: {os.path.abspath(output_filename)}")
    # end
    #Added 2/27
    coordinates_saved = False
    while True:
        
        frame = camera.get_frame()
        if frame is None:
            break

        camera_position, processed_frame = detect_markers(frame, marker_queue, camera_matrix, dist_coeffs, 0)

        #Added to try to record, havent test      
        frame = cv2.resize(frame, (frame_width, frame_height))  # Ensure size consistency
        #marker_list = get_detected_markers(frame, camera) #Dont know why this isnt working get_detected_marker is used as detect_markers here
        video_writer.write(frame)  # Write the frame to the output file
        #end

        if camera_position is not None:
            marker_id, tvec = camera_position
           #Added this to see if the coordinates will stop updating once the marker is found TN 2/27
            if not coordinates_saved and marker_id == 0:
                print(f"TVEC in camera_run: {tvec}")
                coordinates_saved = True
            #End
            distance_to_marker_queue.put(tvec)
        # Display camera positions in the corner of the window
        display_camera_position(processed_frame, camera_position)
        
        cv2.imshow("Camera Feed", processed_frame)
        
        if cv2.waitKey(1) == ord('q'):
            break
    
    camera.close()
'''
def dummy_coords ():
    dummy_location = [32.123213, -92.1231231]
    dummy_marker_found = True
    return dummy_location, dummy_marker_found

def dummy_coords2 ():
    dummy_location = [90.123213, -20.1231231]
    dummy_marker_found = True
    return dummy_location, dummy_marker_found
'''

def comms(ser, isMarkerFound, location_queue):
    #Tell coms queue is ready TN 2/28
    comms_ready.set()

    while True:
        # location_queue.put([32.123213, -92.1231231])      # FOR DEBUGGING
        # location_queue.put([90.123213, -20.1231231])      # FOR DEBUGGING
        if not location_queue.empty():
            locationTuple = location_queue.get()
            data = str(locationTuple) + str(isMarkerFound.value) + str("\n")
            ser.write(data.encode('utf-8'))
            # ser.write(data1.encode('utf-8'))              # FOR DEBUGGING
            #print(f"Sent: {data}")
            #logger.avc(f"Sent From Jetson: {data}")
            if ser.in_waiting > 0:
                message = ser.readline().decode('utf-8').strip()
                logger.avc(f"{message}\n")
            if(isMarkerFound.value):
                logger.avc(f"ArUco Marker Found At {str(locationTuple)}")
            time.sleep(1)  # Wait 1 second before sending the next message

if __name__ == "__main__":
    
    #TODO: need to make a graceful start and exit
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
        
        # Events to signal when each process has booted TN 2/28
        camera_ready = multiprocessing.Event()
        search_ready = multiprocessing.Event()
        comms_ready = multiprocessing.Event()

        # Start the flight process
        flight_process = multiprocessing.Process(target=drone_control, args=(location_queue, isMarkerFound, distance_to_marker_queue))
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
      