import serial  # for serial port control of serial devices
import serial.tools.list_ports
import multiprocessing
import sys
import os
import re
from datetime import datetime
sys.path.append("/home/enorda/.local/lib/python3.6/site-packages")  # get the missing packages on the python path
#import pyzed.sl as sl  # for zed camera api
import time
from dronekit import connect, Vehicle, VehicleMode, mavutil, LocationGlobalRelative, LocationGlobal, \
    Command  # for the drone control interface
import argparse
import time
import cv2  # for video processing and handling
import cv2.aruco as aruco  # for aruco processing
import numpy as np
import math
from math import radians, cos, sin, sqrt, atan2, atan, tan
import queue
import datetime

#from simple_pid import PID  # library for pid controllers
import dataclasses
from dataclasses import dataclass
# import to do path gen stuff automatically
import folium
import csv
from folium import plugins
import webbrowser  # so folium can make a map display

SIMULATE_DRONE = True

search_speed = 3.0  # m/s speed to search for marker in waypoint stage
align_speed = 0.65  # m/s speed to align vehicle to marker
max_single_id_search_time = 50  # time in seconds to search for and identified marker before giving up
ALTITUDE = 3.0  # Altitude to take off to (meters)
ALIGN_VEHICLE_TO_MARKER_ALT = 1.25  # meters to align vehicle at eventually
FRAME_SIZE_OVERLAP = 0.9  # overlap of the frames in the search pattern
# pids for both axis
output_min = -40  # min output for pid loop
output_max = 40  # max output for pid loop
# # initialize the pid loop for the gimbal pitch axis
# pitch_pid = PID(3.0, 3.0, 0.27, setpoint=0, output_limits=(output_min, output_max))
# # initialize the pid loop for the gimbal roll axis
# roll_pid = PID(3.75, 3.5, 0.24, setpoint=0, output_limits=(output_min, output_max))
# # initialize the pid loop for the x axis alignment
# x_align_pid = PID(1.0, 1.0, 0.05, setpoint=0, output_limits=(-10, 10))
# # initialize the pid loop for the y axis alignment
# y_align_pid = PID(1.0, 1.0, 0.05, setpoint=0, output_limits=(-10, 10))

# angle error array initialize
error_window_length = 5  # length of the error window
error_init_number = 1000  # starting error value for the list
error_angle_data_points = [error_init_number] * error_window_length  # make the specified list with the init numbers

# xy error array initialize
error_init_number = 1000  # starting error value for the list
error_xy_data_points = [error_init_number] * error_window_length  # make the specified list with the init numbers
max_queue_size = 1  # max number of items in the queue
xy_error_threshold_stage1 = 1.15  # the threshold under which to begin dropping altitude while aligning
xy_error_threshold_stage2 = 0.275  # the threshold under which to stop aligning vehicle, and begin engaging

# Set angles for each servo and the valve (0 to 180 degrees), 90 is horizontal
servo1_angle = 0.0  # unused
servo2_angle = 90.0  # roll
servo3_angle = 90.0  # pitch
valve_output_state = 0
valve_open_time = 1500.0
max_pitch_angle = 125.0
max_roll_angle = 125.0
min_pitch_angle = 55.0
min_roll_angle = 55.0

# set targeting thresholds and Variables
marker_length = 0.1605  # in meters
friendly_marker_id = 0  # id of the friendly aruco marker
number_of_markers_to_engage = 5  # max number of markers to shoot before going to the finish line, includes freindlyas already engaged
number_of_markers_engaged = 0  # total number of markers that have been shot in this flight
marker_id_status = [False] * 1000  # create an array of 1000 marker ids, index = id, and set the status to false
marker_id_status[friendly_marker_id] = True  # Tell it we have already engaged the friendly marker so dont engage
max_engagement_angle = 12  # maximum angle error avg to shoot water
marker_to_engage = -1
ALLOWABLE_MARKER_MISSING_TIME = 5  # max time marker is allowed to be out of frame before returning to last known location

# camera settings and Data log
camera_exposure_value = 5  # the value of the exposure time for each frame, as a percentage of fps, idk its weird
#video_name_header = "uav_video_data_log_"  # video log header name
directory = "/home/enorda/Desktop/Auto12"
video_start_time = None  # video start time to calc ellapsed video time

# rtx logging stuff
uav_rtx_log_folder_path = "/home/enorda/Desktop/Auto12/rtxLogs"
uav_rtx_log_file_header = 'uav_engagement_data_log'

fence_waypoint_array = []  # store the fence array waypoints

# functions for search algo
def generate_folium_map(waypoints, fence_waypoint_array):
    global home_location
    global home_latitude
    global home_longitude
    # Create a Folium map centered on home location
    my_map = folium.Map(location=[waypoints[0][0], waypoints[0][1]], zoom_start=20, max_zoom=25)

    """
    # add a manually acquired image overlay to the map
    folium.raster_layers.ImageOverlay(
        image="file:///C:/Users/Admin/PycharmProjects/TestUartUax/Stuff/lot.jpg",
        bounds=[[folium_bbox[1], folium_bbox[0]], [folium_bbox[3], folium_bbox[2]]],
        opacity=0.7,
        name='Landsat Image Overlay'
    ).add_to(my_map)
    """
    print(fence_waypoint_array)

    thefencepoint = (fence_waypoint_array[0][1], fence_waypoint_array[0][2])
    print(f"fence: {thefencepoint}")
    folium.Marker(thefencepoint, popup=f'Fencepoint No. {1}: {thefencepoint}', icon=folium.Icon(color='blue')).add_to(
        my_map)

    thefencepoint = (fence_waypoint_array[1][1], fence_waypoint_array[1][2])
    print(f"fence: {thefencepoint}")
    folium.Marker(thefencepoint, popup=f'Fencepoint No. {2}: {thefencepoint}', icon=folium.Icon(color='blue')).add_to(
        my_map)

    thefencepoint = (fence_waypoint_array[2][1], fence_waypoint_array[2][2])
    print(f"fence: {thefencepoint}")
    folium.Marker(thefencepoint, popup=f'Fencepoint No. {3}: {thefencepoint}', icon=folium.Icon(color='blue')).add_to(
        my_map)

    thefencepoint = (fence_waypoint_array[3][1], fence_waypoint_array[3][2])
    print(f"fence: {thefencepoint}")
    folium.Marker(thefencepoint, popup=f'Fencepoint No. {4}: {thefencepoint}', icon=folium.Icon(color='blue')).add_to(
        my_map)

    # Create a PolyLine
    polyline = folium.PolyLine(locations=waypoints, color='blue', weight=3, opacity=0.0).add_to(my_map)

    # Add arrows to the PolyLine
    plugins.PolyLineTextPath(
        polyline,
        '\u2192',  # Unicode arrow character (right arrow)
        repeat=True,
        offset=6,
        attributes={'fill': 'red', 'font-weight': 'bold', 'font-size': '35'}
    ).add_to(my_map)

    '''
    new_waypoint_index = 0
    # Add waypoints to the map
    for waypoint in waypoints:
        new_waypoint_index += 1
        print(f"waypoint: {waypoint}")
        folium.Marker(waypoint, popup=f'Waypoint No. {new_waypoint_index}: {waypoint}',
                      icon=folium.Icon(color='green')).add_to(my_map)
    '''
    # Save the map as a html file
    my_map.save('generated_search_pattern_waypoints.html')

    # Optionally, open the HTML file in a web browser
    open_in_browser = True
    if open_in_browser is True:
        webbrowser.open('generated_search_pattern_waypoints.html')

    return


def equirectangular_approximation(coord1, coord2,alt = None):
    """
    Calculate the approximate straight-line distance between two GPS coordinates using the Equirectangular approximation.
    Parameters:
    - coord1: Tuple of (latitude, longitude) for the first point.
    - coord2: Tuple of (latitude, longitude) for the second point.
    Returns:
    - Distance in kilometers.
    """
    # Earth radius in kilometers
    R = 6371.0
    # Convert latitude and longitude from degrees to radians
    lat1, lon1 = map(radians, coord1)
    lat2, lon2 = map(radians, coord2)
    # Equirectangular approximation
    x = (lon2 - lon1) * cos((lat1 + lat2) / 2)
    y = lat2 - lat1
    # Return Distance in kilometers
    if alt is not None:
        distance = R * math.sqrt(x ** 2 + y ** 2)+ abs(alt/1000.0)
    else:
        distance = R * math.sqrt(x ** 2 + y ** 2)
    return distance


def save_waypoints_to_csv(waypoints, csv_filename):
    waypoint_index = 0
    with open(csv_filename, 'w', newline='') as csvfile:
        # create and write the header
        fieldnames = ['latitude', 'longitude']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        # run thru each waypoint and add to the csv file
        for waypoint in waypoints:
            waypoint_index += 1
            writer.writerow({'latitude': waypoint[0], 'longitude': waypoint[1]})  # add waypoint to file


def generate_zigzag_waypoints(bottom_left, top_right, rows, cols):
    zig_waypoints = []
    # Calculate the step size for rows and columns
    row_step = (top_right[0] - bottom_left[0]) / (rows - 1)
    col_step = (top_right[1] - bottom_left[1]) / (cols - 1)

    for i in range(rows):
        # Adjust the column order for every other row to create a zigzag pattern
        col_range = range(cols) if i % 2 == 0 else reversed(range(cols))

        for j in col_range:
            lat = bottom_left[0] + i * row_step
            lon = bottom_left[1] + j * col_step
            zig_waypoints.append((lat, lon))

    return zig_waypoints


def load_waypoints_from_csv(file_path):
    global ALTITUDE
    csv_loaded_waypoints = []
    with open(file_path, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            latitude = float(row['latitude'])
            longitude = float(row['longitude'])
            #altitude = float(ALTITUDE)
            #csv_loaded_waypoints.append(LocationGlobalRelative(latitude, longitude, altitude))
            csv_loaded_waypoints.append([latitude, longitude])
    return csv_loaded_waypoints


def waypoint_distance(current_location, target_location):
    dlat = target_location.lat - current_location.lat
    dlon = target_location.lon - current_location.lon
    return (dlat ** 2 + dlon ** 2) ** 0.5 * 1e5


@dataclass
class HomeClassLoc:  # define a class to handle the new home location
    lat: float
    lon: float


def generate_zig_zag_path_waypoints(point1, point2, point3, num_cols, num_rows):
    # Calculate the side vectors
    side_vector1 = np.array(point2) - np.array(point1)
    side_vector2 = np.array(point3) - np.array(point2)

    # Calculate the angle between the sides
    dot_product = np.dot(side_vector1, side_vector2)
    norm_product = np.linalg.norm(side_vector1) * np.linalg.norm(side_vector2)
    cosine_angle = dot_product / norm_product
    angle = np.arccos(np.clip(cosine_angle, -1.0, 1.0))

    # Choose the side with an angle greater than 180 degrees so the parallelogram doesnt cross itself
    if angle > 180:
        fourth_point = (point3[0] + side_vector1[0], point3[1] + side_vector1[1])
    else:
        fourth_point = (point1[0] + side_vector2[0], point1[1] + side_vector2[1])

    x = math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)  # Width of the grid
    y = math.sqrt((point2[0] - point3[0]) ** 2 + (point2[1] - point3[1]) ** 2)  # Height of the grid

    print(x)
    print(y)

    # Define the corners of the rectangle
    rectangle = np.array([
        [0, 0],  # Bottom-left corner
        [x, 0],  # Bottom-right corner
        [x, y],  # Top-right corner
        [0, y]  # Top-left corner
    ])

    # Define the corners of the known parallelogram
    parallelogram = np.array([
        [point1[0], point1[1]],  # Bottom-left corner
        [point2[0], point2[1]],  # Bottom-right corner
        [point3[0], point3[1]],  # Top-right corner
        [fourth_point[0], fourth_point[1]]  # Top-left corner
    ])

    # Create coordinate vectors for the columns and rows
    cols = np.linspace(0, x, num_cols)
    rows = np.linspace(0, y, num_rows)

    # Create the meshgrid
    col_mesh, row_mesh = np.meshgrid(cols, rows, indexing='xy')

    # Flatten the meshgrid arrays and combine them into one array of xy pairs
    grid_points = np.column_stack((col_mesh.ravel(), row_mesh.ravel()))

    # reverse every other row in the array
    # n = int(math.floor(x) / col_spacing)
    n = num_cols
    result = grid_points.copy()
    for i in range(0, len(grid_points), n * 2):
        result[i:i + n] = np.flip(result[i:i + n], axis=0)

    grid_points = result

    # Convert rectangle and parallelogram to numpy arrays
    rectangle = np.array(rectangle)
    parallelogram = np.array(parallelogram)

    # Append a column of ones to the rectangle coordinates for the translation component
    rectangle_with_ones = np.hstack([rectangle, np.ones((4, 1))])

    # Find the transformation matrix that maps the rectangle onto the parallelogram
    transform_matrix, _, _, _ = np.linalg.lstsq(rectangle_with_ones, parallelogram, rcond=None)

    # Apply the transformation matrix to the rectangle
    skewed_rectangle = np.dot(rectangle_with_ones, transform_matrix)

    # Append a column of ones to the rectangle coordinates for the translation component
    grid_with_ones = np.hstack([grid_points, np.ones((len(grid_points), 1))])

    skewed_grid = np.dot(grid_with_ones, transform_matrix)
    print(skewed_grid)
    skewed_rectangle, skewed_grid_points = skewed_rectangle[:, :2], skewed_grid[:,
                                                                    :2]  # Exclude the last column of ones

    skewed_grid_points = np.flip(skewed_grid_points, axis=0)

    return skewed_grid_points


def run_path_generation(vehicle, frame_width_meters, frame_height_meters):
    global home_location
    global fence_waypoint_array
    global home_latitude
    global home_longitude
    global home_altitude
    # Wait for the vehicle to have a GPS fix

    while not vehicle.gps_0.fix_type:
        print("Waiting for the GPS to have a fix...")
        time.sleep(1)
    print("gpsfixed")

    # Download the vehicle waypoints (commands). Wait until download is complete.
    print("getting the cmds position")

    # Request the total number of mission items
    vehicle.commands.download()

    # Wait for the mission download to complete
    vehicle.commands.wait_ready()

    # Get the list of downloaded mission items
    cmds = vehicle.commands
    print(f"cmds: {cmds}")

    if SIMULATE_DRONE is True:
        # vehicle.home_location = LocationGlobal(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_frame.alt)
        vehicle.home_location = LocationGlobal(31, 31, 180)
    # Print waypoint data
    if len(cmds) > 0:
        print("Mission commands:")
        for cmd in cmds:
            print(f"{cmd.seq}, {cmd.x}, {cmd.y}, {cmd.z}")
            fence_waypoint_array.append([cmd.seq, cmd.x, cmd.y, cmd.z])
    else:
        print("No mission commands downloaded.")
        sys.exit("Exiting due to no waypoint data from cmds")

    # Wait for the home location to be set
    while vehicle.home_location is None:
        print("Waiting for the home location to be set...")
        time.sleep(1)

    # Retrieve home location
    home_location = vehicle.home_location

    # home location is not very useful because it changes to current loc every arming
    # Parse and print home location information
    if home_location is not None:
        home_latitude = home_location.lat
        home_longitude = home_location.lon
        home_altitude = home_location.alt
        print(f"Home Location: Latitude={home_latitude}, Longitude={home_longitude}, Altitude={home_altitude}")
    else:
        print("Home location is not available.")
        sys.exit("Exiting due to no home position")

    # print the fence data defined as 4 normal waypoints in mission planner

    # Specify the rectangular area with four corners defined
    top_left_corner = (fence_waypoint_array[0][1], fence_waypoint_array[0][2])  # all copied from mission planner
    top_right_corner = (fence_waypoint_array[1][1], fence_waypoint_array[1][2])
    bottom_right_corner = (fence_waypoint_array[2][1], fence_waypoint_array[2][2])
    bottom_left_corner = (fence_waypoint_array[3][1], fence_waypoint_array[3][2])

    landing_zone_waypoint = (fence_waypoint_array[4][1], fence_waypoint_array[4][2])  # The location to land the drone at after finishing search and shoot

    # get width and length of the search area in meters
    horizontal_distance = equirectangular_approximation(top_left_corner, top_right_corner) * 1000
    vertical_dist = equirectangular_approximation(top_right_corner, bottom_right_corner) * 1000

    # Number of rows and columns in the zigzag grid based on the size of the field and radius of points
    try:
        cols = int(horizontal_distance // (frame_width_meters * FRAME_SIZE_OVERLAP))
        rows = int(vertical_dist // (frame_height_meters * FRAME_SIZE_OVERLAP))
    except Exception as e:
        print("rows and cols too smol using 2 atleast")
        cols = 2
        rows = 2

    if rows < 2 or cols < 2:
        cols = 2
        rows = 2

    rows = 2

    # Generate zigzag waypoints
    # waypoints = generate_zigzag_waypoints(bottom_left_corner, top_right_corner, rows, cols)
    waypoints = generate_zig_zag_path_waypoints(top_left_corner, top_right_corner, bottom_right_corner, rows, cols)

    # Save waypoints to CSV
    csv_filename = 'generated_search_pattern_waypoints.csv'
    save_waypoints_to_csv(waypoints, csv_filename)

    print(f'Waypoints saved to {csv_filename}')

    # should the Code generate a HTML map also?
    genMap = True
    if genMap is True:
        generate_folium_map(waypoints, fence_waypoint_array)

    HomeClassLoc.lat = home_latitude
    HomeClassLoc.lon = home_longitude

    # Load waypoints from CSV file
    waypoints = load_waypoints_from_csv('generated_search_pattern_waypoints.csv')

    return waypoints, top_left_corner, top_right_corner, landing_zone_waypoint

# Used to connect to copter with args from command line
def connectMyCopter():
    if SIMULATE_DRONE:
        # Create a SITL drone instance instead of launching one beforehand
        import dronekit_sitl
        sitl = dronekit_sitl.start_default(32.92019271850586, -96.9479751586914)
        connection_string = sitl.connection_string()
        #vehicle = connect(connection_string, wait_ready=True)
        vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

        

    else:
        vehicle = connect('/dev/ ', baud=115200, wait_ready=True)
        global ser
        ser = serial.Serial('/dev/ttyACM1', 115200)

    return vehicle

def arm_drone(vehicle: Vehicle):
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
        print("Waiting for drone to arm")
        time.sleep(1)  # Wait one second before checking if drone has been armed
    print("The drone has now been armed")

    # Check if GPS is functioning
    while vehicle.gps_0.fix_type < 2:  # Ensure GPS is ready
        print(" Waiting for GPS to initialise...", vehicle.gps_0.fix_type)
        time.sleep(1)
    print("Copter GPS Ready")


# Used to take off the drone to a specific altitude
def takeoff_drone(vehicle: Vehicle, targetAltitude: float):
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

cornerCoordinates = [
    [1, 32.920440673828125, -96.94830322265625],
    [2, 32.920440673828125, -96.9479751586914],
    [3, 32.92019271850586, -96.9479751586914],
    [4, 32.92019271850586, -96.94831085205078]
]
wpCoords = load_waypoints_from_csv('./generated_search_pattern_waypoints.csv')

if __name__ == "__main__":
     #generate_folium_map(wpCoords,cornerCoordinates)
     TAKEOFF = True
     myCopter = connectMyCopter()
     if TAKEOFF:
        arm_drone(myCopter)
        search_waypoints = load_waypoints_from_csv('generated_search_pattern_waypoints.csv')
        takeoff_drone(myCopter, 4)
        point1 = LocationGlobalRelative(32.92019271850586, -96.9479751586914, 4)
        myCopter.simple_goto(point1, groundspeed=10)
        time.sleep(300)
        # for wp in search_waypoints:
        #     myCopter.simple_goto(LocationGlobal(wp[0],wp[1],4), groundspeed=5)
        #     print("Going to: ", LocationGlobal(wp[0],wp[1],4))
        
       










#generate geofence
#generate waypoints
#takeoff
#fly to waypoint
#landing