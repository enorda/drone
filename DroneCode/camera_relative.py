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
from cameraprocess import *

# Set up option parsing to get connection string
import argparse
import logging
import cv2
import cv2.aruco as aruco
import numpy as np
#Added this to save video image
import os

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

logging.basicConfig(
    filename='flight.log',
    level=AVC_LOG,  # Set to the custom level
    format='%(asctime)s - %(levelname)s - %(message)s'
)
root_logger = logging.getLogger()
for handler in root_logger.handlers:
    handler.addFilter(CustomLevelFilter())
logger = logging.getLogger("FlightLogger")

# Function to generate a unique filename
def get_unique_filename(base_filename):
    """
    This function checks if the file already exists.
    If it does, it appends a number to the filename to make it unique.
    """
    file_name, file_extension = os.path.splitext(base_filename)
    counter = 1
    new_filename = base_filename
    while os.path.exists(new_filename):
        new_filename = f"{file_name}_{counter}{file_extension}"
        counter += 1
    return new_filename

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
unique_output_filename = get_unique_filename('flight_output.txt')

# Set sys.stdout to your custom output class to log to both terminal and file
sys.stdout = Output(unique_output_filename)

# Set up logging to log both to flight.log and a text file
# Set up a StreamHandler to log to the unique text file
text_file_handler = logging.FileHandler(unique_output_filename, mode='w')  # 'w' to overwrite the file every time
text_file_handler.setLevel(AVC_LOG)  # Set to the custom level

# Add formatter for the text file handler
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
text_file_handler.setFormatter(formatter)

# Add the text file handler to the logger
root_logger.addHandler(text_file_handler)


USING_ZED_CAMERA = True  # Set to True if using the ZED camera, False otherwise
espPORT = '/dev/ttyUSB0'  # Change to your actual port
espBAUDRATE = 115200  # Ensure this matches the ESP32 baud rate
frame_width = 1280
frame_height = 720
#Added this to grab frames for video
fps = 30
#end

parser = argparse.ArgumentParser(description="Connect to a drone.")
#parser.add_argument("--livedrone", action="store_true", help="Connect to a real drone instead of simulating.")
args = parser.parse_args()





def camera_run(marker_queue, distance_to_marker_queue):
    camera = Camera(USING_ZED_CAMERA, frame_width, frame_height)
    camera_matrix, dist_coeffs = load_calibration(CALIBRATION_FILE_PATH)
    cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL) #Added this to make the windows adjustable but havent tested
    
    # Initialize VideoWriter to save video
    output_filename = "relative_position_test.avi"
    fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Codec for AVI file
    video_writer = cv2.VideoWriter(output_filename, fourcc, fps, (frame_width, frame_height))
    print(f"Saving video to: {os.path.abspath(output_filename)}")
    # end
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
            if not coordinates_saved and marker_id == 0:
                print(f"marker found {tvec}")
                coordinates_saved = True
            distance_to_marker_queue.put(tvec)
            print(f"This is the distance queue in IntegratedUAV: {distance_to_marker_queue.get()}")

        # Display camera positions in the corner of the window
        display_camera_position(processed_frame, camera_position)
        
        cv2.imshow("Camera Feed", processed_frame)
        
        if cv2.waitKey(1) == ord('q'):
            break
    
    camera.close()


if __name__ == "__main__":
       # vehicle = connectMyCopter()

        marker_queue = multiprocessing.Queue()
        location_queue = multiprocessing.Queue()
        distance_to_marker_queue = multiprocessing.Queue()
        isMarkerFound = multiprocessing.Value('b', False)
  # Start the camera and search algorithm processes
        camera_process = multiprocessing.Process(target=camera_run, args=(marker_queue, distance_to_marker_queue))
        camera_process.start()
      #
        # Wait for the processes to finish
        camera_process.join()


