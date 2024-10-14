from dronekit import connect, Vehicle, VehicleMode, LocationGlobalRelative
import time
import math
# from colorama import Fore, Style
import cv2
import cv2.aruco as aruco
import serial
import numpy as np
# from pyproj import Proj


ALTITUDE = 3  # Altitude to take off to (meters)
WAIT_TIME = 10  # Time to wait for markers to be found before landing (seconds)
DRONE_SPEED = 0.5  # Speed of the drone (m/s)
FRIENDLY_MARKER_ID = 0  # ID of the friendly marker (our marker is 0)
DISTANCE_THRESHOLD = 0.6  # Distance threshold to target location in the goto_location function (meters)
MARKER_LENGTH = 0.3048  # Length of the aruco marker's side (meters), 0.3048m = 1ft

SIMULATE_DRONE = True  # Set to True to simulate the drone without connecting to a real drone
USING_ZED_CAMERA = not SIMULATE_DRONE  # Set to True if using the ZED camera, False otherwise
TAKEOFF = True  # Set to True to take off, False otherwise


class Camera:
    def __init__(self):
        if USING_ZED_CAMERA:
            global sl
            import pyzed.sl as sl
            self.zed = sl.Camera()
            self.init = sl.InitParameters()
            self.init.camera_resolution = sl.RESOLUTION.HD1080
            self.status = self.zed.open(self.init)
            if self.status != sl.ERROR_CODE.SUCCESS:
                print(f"Error opening ZED camera: {self.status}, is the camera connected?")
                return None
        else:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                print("Error opening the camera")
                return None
        cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)

    def getFrame(self):
        if USING_ZED_CAMERA:
            if self.zed.grab() != sl.ERROR_CODE.SUCCESS:
                print("Error grabbing frame from ZED camera")
                return None
            else:
                image = sl.Mat()
                self.zed.retrieve_image(image, sl.VIEW.LEFT)
                frame = image.get_data()
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
                return frame
        else:
            ret, frame = self.cap.read()
            if not ret:
                print("Error grabbing frame from camera")
                return None
            return frame

    def close(self):
        if USING_ZED_CAMERA:
            self.zed.close()
        else:
            self.cap.release()


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
        global ser
        ser = serial.Serial('/dev/ttyACM1', 115200)

    return vehicle


# Used to arm the drone
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


# Used to land the drone, prints height every second while descending
def land_drone(vehicle: Vehicle):
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
def goto_location(vehicle: Vehicle, lat: float, lon: float, alt: float):
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


def goto_marker(vehicle: Vehicle, id: int, marker_list: dict):
    if marker_list.keys().__contains__(id):
        x, y = marker_list[id][1][0][0], marker_list[id][1][0][1]
        print(f"Vector to marker {id}: ({x}, {y})")
        x, y = meters_to_wgs84degrees(vehicle, x, y)
        if x is not None and y is not None:
            print(f"Moving to marker {id}")
            goto_location(vehicle, vehicle.location.global_relative_frame.lat + x, vehicle.location.global_relative_frame.lon + y, ALTITUDE)
    else:
        print(f"Marker {id} not found")


def meters_to_wgs84degrees(vehicle: Vehicle, x: float, y: float):
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


def wgs84_distance_to_meters(lat1: float, lon1: float, lat2: float, lon2: float):
    # Calculate the distance between two WGS 84 coordinates in meters

    # Define the projection
    p = Proj(proj='utm', zone=33, ellps='WGS84')

    # Convert the coordinates to UTM
    x1, y1 = p(lon1, lat1)
    x2, y2 = p(lon2, lat2)

    # Calculate the distance
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


# Function to get the detected markers from the camera (returns a dictionary of marker IDs and their poses)
def get_detected_markers(camera: Camera):
    # cameraMatrix = np.array([[1087.295120358312, 0.0, 672.5803974446591], [0.0, 904.9501417964135, 152.10434756756268], [0.0, 0.0, 1.0]])
    # distortionCoef = np.array([[0.8828484612711133, 2.1668104320308754, -0.25913938950617754, -0.2070873886137139, -53.533897089498666]])

    frame = camera.getFrame()
    # frame = cv2.undistort(frame, cameraMatrix, distortionCoef)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # show the frame
    cv2.imshow("Camera Feed", frame)
    cv2.waitKey(1)

    # Initialize the aruco dictionary and parameters
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
    # parameters = aruco.DetectorParameters_create()

    # Initialize the dictionary of detected markers
    detected_markers = {}

    # Detect the markers in the frame
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)  # , parameters=parameters)

    # If markers are detected, estimate the pose
    if ids is not None:
        # Show the detected markers
        frame = aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.imshow("Camera Feed", frame)
        cv2.waitKey(1)

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, MARKER_LENGTH, None, None)

        # Loop over the detected markers
        for i in range(len(ids)):
            # Add the detected marker to the dictionary
            detected_markers[ids[i][0]] = (rvecs[i], tvecs[i])

    # Return the dictionary of detected markers
    print(f"Detected markers: {detected_markers}")
    return detected_markers


def writeArduinoSerial(servo1_angle_f: int, servo2_angle_f: int, servo3_angle_f: int, valve_output_state_f: int, valve_open_time_f: int):
    """
    Function to send commands to the Arduino, which controls the water jet. The function takes in the following parameters:
     - servo1_angle_f: The angle of the first servo (N/A)
     - servo2_angle_f: The angle of the second servo (Roll)
     - servo3_angle_f: The angle of the third servo (Pitch)
     - valve_output_state_f: The state of the valve (0 for closed, 1 for open)
     - valve_open_time_f: The time the valve should be open for (in milliseconds)

     Roll reference: 90 degrees points straight down, 180 points forward, 0 points backward (towards the drone)
    """
    if not SIMULATE_DRONE:
        print(f"Sending command, N/A:{servo1_angle_f}, Roll:{servo2_angle_f}, Pitch:{servo3_angle_f}, ValveState:{valve_output_state_f}, OpenTime:{valve_open_time_f}/n")
        ser.write(f"A {servo1_angle_f} {servo2_angle_f} {servo3_angle_f} {valve_output_state_f} {valve_open_time_f}/n".encode())
        print("Command sent")
    else:
        print("Arduino command not sent (simulation mode)")


if __name__ == "__main__":
    #  Basic takeoff sequence
    myCopter = connectMyCopter()
    camera = Camera()

    if TAKEOFF:
        arm_drone(myCopter)
        takeoff_drone(myCopter, ALTITUDE)

    # Create window to display camera feed (1920x1080 resolution)
    cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Camera Feed", 1920, 1080)

    # Loop until either a marker is found or the allotted time has passed
    start_time = time.time()
    while True: # time.time() - start_time < WAIT_TIME:
        marker_list = get_detected_markers(camera)
        #  If markers are detected
        if marker_list.__len__() > 0:
            # If friendly marker detected
            if marker_list.keys().__contains__(FRIENDLY_MARKER_ID):
                print("Found friendly marker")
                # goto_marker(myCopter, FRIENDLY_MARKER_ID, marker_list)
                # break
            # If rival marker detected
            else:
                print("Found rival marker")
                # goto_marker(myCopter, list(marker_list.keys())[0], marker_list)
                # break

    if TAKEOFF:
        land_drone(myCopter)

    print(f"Markers found: {marker_list.keys()}")
    myCopter.close()
    camera.close()
    time.sleep(3)
    cv2.destroyAllWindows()

    if not SIMULATE_DRONE:
        ser.close()
