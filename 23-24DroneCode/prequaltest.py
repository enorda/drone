from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
# from colorama import Fore, Style
import cv2
import cv2.aruco as aruco
import serial
import numpy as np
# from pyproj import Proj


ALTITUDE = 2  # Altitude to take off to (meters)
WAIT_TIME = 10  # Time to wait for markers to be found before landing (seconds)
DRONE_SPEED = 0.5  # Speed of the drone (m/s)
FRIENDLY_MARKER_ID = 0  # ID of the friendly marker (our marker is 0)
DISTANCE_THRESHOLD = 0.6  # Distance threshold to target location in the goto_location function (meters)
MARKER_LENGTH = 0.3048  # Length of the aruco marker's side (meters), 0.3048m = 1ft

SIMULATE_DRONE = False  # Set to True to simulate the drone without connecting to a real drone
USING_ZED_CAMERA = True  # Set to True if using the ZED camera, False otherwise


# Used to connect to copter with args from command line
def connectMyCopter():
    if SIMULATE_DRONE:
        # Create a SITL drone instance instead of launching one beforehand
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()
        vehicle = connect(connection_string, wait_ready=True)

    else:
        vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True)

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
        print("Waiting for drone to arm")
        time.sleep(1)  # Wait one second before checking if drone has been armed
    print("The drone has now been armed")
    time.sleep(3)
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


# Used to land the drone, prints height every second while descending
def land_drone(vehicle):
    print("Setting copter into LAND mode")
    vehicle.mode = VehicleMode('LAND')
    while vehicle.mode != 'LAND':
        time.sleep(1)
    print("Initiating landing now...")

    while vehicle.armed:  # While the drone has not landed
        currDroneHeight = vehicle.location.global_relative_frame.alt
        print("Current drone elevation: ", currDroneHeight)
        time.sleep(1)

    print("The copter has landed!")


# Function to move the drone to a specific location (global relative)
def goto_location(vehicle, lat, lon, alt):
    target_location = LocationGlobalRelative(lat, lon, alt)
    timeout_time = time.time() + ((wgs84_distance_to_meters(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, lat, lon) / DRONE_SPEED) * 1.5)
    vehicle.simple_goto(target_location, airspeed=DRONE_SPEED)
    while True:
        time.sleep(1)
        distance = wgs84_distance_to_meters(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, lat, lon)
        print(f"Distance to target: {distance} meters")
        if distance < DISTANCE_THRESHOLD:  # Adjust this threshold as needed
            break
        if time.time() > timeout_time:
            print("goto_location timed out")
            break


def goto_marker(vehicle, id: int, marker_list):
    if marker_list.keys().__contains__(id):
        x, y = marker_list[id][1][0][0], marker_list[id][1][0][1]
        print(f"Vector to marker {id}: ({x}, {y})")
        x, y = meters_to_wgs84degrees(vehicle, x, y)
        if x is not None and y is not None:
            print(f"Moving to marker {id}")
            goto_location(vehicle, vehicle.location.global_relative_frame.lat + x, vehicle.location.global_relative_frame.lon + y, ALTITUDE)
    else:
        print(f"Marker {id} not found")


def meters_to_wgs84degrees(vehicle, x: float, y: float):
    # Converts a (x, y) offset in meters to an offset in WGS 84 coordinates, given the current location of the drone

    # Define the projection
    p = Proj(proj='utm', zone=33, ellps='WGS84')

    # Convert the current location to UTM coordinates
    utm_x, utm_y = p(vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.lat)

    # Convert the offset to UTM coordinates
    utm_x += x
    utm_y += y

    # Convert the UTM coordinates back to WGS 84 coordinates
    lon, lat = p(utm_x, utm_y, inverse=True)

    # Return the offset in WGS 84 coordinates
    return lat - vehicle.location.global_relative_frame.lat, lon - vehicle.location.global_relative_frame.lon


def wgs84_distance_to_meters(lat1, lon1, lat2, lon2):
    # Calculate the distance between two WGS 84 coordinates in meters

    # Define the projection
    p = Proj(proj='utm', zone=33, ellps='WGS84')

    # Convert the coordinates to UTM
    x1, y1 = p(lon1, lat1)
    x2, y2 = p(lon2, lat2)

    # Calculate the distance
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


# Function to get the detected markers from the camera (returns a dictionary of marker IDs and their poses)
def get_detected_markers():
    # Initialize the camera and the frame
    if USING_ZED_CAMERA:
        import pyzed.sl as sl
        zed = sl.Camera()
        init = sl.InitParameters()
        init.camera_resolution = sl.RESOLUTION.HD1080
        status = zed.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            print(f"Error opening ZED camera: {status}, is the camera connected?")
            return {}

        image = sl.Mat()
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)
            frame = image.get_data()
        else:
            print("Error grabbing frame from ZED camera")
            return {}
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    else:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Error opening the camera")
            return {}
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Initialize the aruco dictionary and parameters
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
    # parameters = aruco.DetectorParameters_create()

    # Initialize the dictionary of detected markers
    detected_markers = {}

    # Detect the markers in the frame
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)

    # If markers are detected, estimate the pose
    if ids is not None:
        cameraMatrix = np.array([[1087.295120358312, 0.0, 672.5803974446591], [0.0, 904.9501417964135, 152.10434756756268], [0.0, 0.0, 1.0]])
        distortionCoef = np.array([[0.8828484612711133, 2.1668104320308754, -0.25913938950617754, -0.2070873886137139, -53.533897089498666]])

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, MARKER_LENGTH, cameraMatrix, distortionCoef)

        # Loop over the detected markers
        for i in range(len(ids)):
            # Add the detected marker to the dictionary
            detected_markers[ids[i][0]] = (rvecs[i], tvecs[i])
        
        print(f"Detected markers: {detected_markers}")

    # Release the camera
    if USING_ZED_CAMERA:
        zed.close()
    else:
        cap.release()

    # Return the dictionary of detected markers
    
    return detected_markers

def live_feed():
	# Create OpenCV window
	cv2.namedWindow("ZED", cv2.WINDOW_NORMAL)

	# Capture frames and perform ArUco marker detection

	image_size = zed.get_camera_information().camera_configuration.resolution
	image_size.width = image_size.width /2
	image_size.height = image_size.height /2
	# Create an RGBA sl.Mat object
	image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
	# Retrieve data in a numpy array with get_data()
	while True:
		if zed.grab() == sl.ERROR_CODE.SUCCESS:
			# Retrieve the left image in sl.Mat
			zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
			# Use get_data() to get the numpy array
			image_ocv = image_zed.get_data()
			image_ocv = cv2.cvtColor(image_ocv, cv2.COLOR_RGBA2RGB)

			# Convert to grayscale
			gray = cv2.cvtColor(image_ocv, cv2.COLOR_BGR2GRAY)

			# Detect ArUco markers
			dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
			corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, dictionary)
			print(corners)
			# Draw detected markers
			if ids is not None and image_ocv is not None:
				print(ids)
				aruco.drawDetectedMarkers(image_ocv, corners,ids)
			
			
			
			# Display frame
			cv2.imshow("ZED", image_ocv)

		key = cv2.waitKey(10)
		if key == 27:  # ESC key to exit
			break

if __name__ == "__main__":
    #  Basic takeoff sequence
    print("Connecting...")
    myCopter = connectMyCopter()
    print("Drone connected")
    arm_drone(myCopter)
    takeoff_drone(myCopter, ALTITUDE)
    
    # live_feed()

    # Loop until either a marker is found or the allotted time has passed
    start_time = time.time()
    while time.time() - start_time < WAIT_TIME:
        marker_list = get_detected_markers()
        #  If markers are detected
        if marker_list.__len__() > 0:
            # If friendly marker detected
            if marker_list.keys().__contains__(FRIENDLY_MARKER_ID):
                print(f"Found friendly marker")
                # goto_marker(myCopter, FRIENDLY_MARKER_ID, marker_list)
                break
            # If rival marker detected
            else:
                print(f"Found rival marker")
                # goto_marker(myCopter, list(marker_list.keys())[0], marker_list)
                break

    land_drone(myCopter)
    myCopter.close()
