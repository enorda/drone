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
from serial import Serial


# Set up option parsing to get connection string
import argparse
import logging
import cv2
import cv2.aruco as aruco
import numpy as np

logging.basicConfig(filename='flight.log',   # Name of the log file
                    level=logging.DEBUG,    # Minimum logging level (DEBUG, INFO, etc.)
                    format='%(asctime)s - %(levelname)s - %(message)s')  # Format for log message

USING_ZED_CAMERA = True  # Set to True if using the ZED camera, False otherwise

espPORT = '/dev/ttyCH341USB0'  # Change to your actual port
espBAUDRATE = 115200  # Ensure this matches the ESP32 baud rate


class Camera:
    def __init__(self):
        print("Initializing camera...")

        if USING_ZED_CAMERA:
            global sl
            import pyzed.sl as sl
            self.zed = sl.Camera()
            self.init = sl.InitParameters()
            self.init.camera_resolution = sl.RESOLUTION.HD1080
            self.init.depth_mode = sl.DEPTH_MODE.NONE
            self.zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 1)
            self.status = self.zed.open(self.init)
            if self.status != sl.ERROR_CODE.SUCCESS:
                print(f"Error opening ZED camera: {self.status}, is the camera connected?")
                return None

        else:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                print("Error opening the camera")
                return None
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1000)  # Adjust this as needed
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Camera Feed", 1000, 720)
        print("Camera initialized")

    def getFrame(self):
        if USING_ZED_CAMERA:
            if self.zed.grab() != sl.ERROR_CODE.SUCCESS:
                print("Error grabbing frame from ZED camera")
                return None
            else:
                image = sl.Mat()
                self.zed.retrieve_image(image, sl.VIEW.LEFT)
                frame = image.get_data()
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)
                return frame
        else:
            ret, frame = self.cap.read()
            if not ret:
                print("Error grabbing frame from camera")
                return None
            return frame

    def showFrame(self, frame):
        cv2.imshow("Camera Feed", frame)

    def getFrameAndShow(self):
        frame = self.getFrame()
        self.showFrame(frame)
        return frame

    def close(self):
        if USING_ZED_CAMERA:
            self.zed.close()
        else:
            self.cap.release()
        cv2.destroyAllWindows()


def get_detected_markers(frame, marker_queue, camera: Camera = None):
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    # Initialize the aruco dictionary and parameters
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)

    # Detect the markers in the frame
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)  # , parameters=parameters)

    if ids is not None:
        # Show the detected markers
        frame = aruco.drawDetectedMarkers(frame, corners)

        for i, marker_id in enumerate(ids.flatten()):
            corner = corners[i][0]

            if marker_id == 0:
                color = (0,255,0)
                zone_label = "Drop Zone"
            else:
                color = (0,0,255)
                zone_label = "Non-Drop Zone"
            
            cv2.polylines(frame, [corner.astype(int)], isClosed=True, color=color, thickness=3)
            zone_label_position = np.mean(corner, axis=0).astype(int)
            cv2.putText(frame, zone_label, tuple(zone_label_position), cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 2, cv2.LINE_AA)
            id_label_position = (zone_label_position[0], zone_label_position[1] + 50)
            cv2.putText(frame, f"ID: {marker_id}", id_label_position, cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 2, cv2.LINE_AA)

        camera.showFrame(frame)

        for marker_id in ids.flatten():
            marker_queue.put(marker_id)
    else:
        camera.getFrameAndShow()

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

def camera_run(marker_queue):
    camera = Camera()
    fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Codec for AVI file
    video_writer = cv2.VideoWriter("output.avi", fourcc, 30, (1000,720))
    while True:
        get_detected_markers(camera.getFrame(), marker_queue, camera)
        if cv2.waitKey(1) == ord('q'):
            break
    camera.close()
    video_writer.release()  # Ensure the video file is properly closed

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

        # Connect to the drone
        enordaCopter = connectMyCopter()
        marker_queue = multiprocessing.Queue()
        location_queue = multiprocessing.Queue()
        isMarkerFound = multiprocessing.Value('b', False)

        # Start the camera and search algorithm processes
        camera_process = multiprocessing.Process(target=camera_run, args=(marker_queue,))
        camera_process.start()

        search_process = multiprocessing.Process(target=search_algorithm, args=(marker_queue, isMarkerFound))
        search_process.start()

        # Start the comms process, passing the serial connection
        comms_process = multiprocessing.Process(target=comms, args=(ser, isMarkerFound, location_queue))
        comms_process.start()

        # Start the flight process
        flight_process = multiprocessing.Process(target=drone_control, args=(location_queue,))
        flight_process.start()

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