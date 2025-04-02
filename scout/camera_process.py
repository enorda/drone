import cv2
import cv2.aruco as aruco
import numpy as np
import pyzed as sl
from math import sqrt, tan, radians
from dronekit import Vehicle, LocationGlobalRelative

# Project file imports
from search_alg import geodesic, Point

# Ensures compiler can find DroneTest module from this file
import sys, os
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(project_root)

CALIBRATION_FILE_PATH = "../CameraCalibration/calibration_chessboard.yaml"  # Path to your calibration file
MARKER_SIZE = 0.1  # Marker size in meters
RVEC, TVEC = None, None


class Camera:
    def __init__(self, using_zed_camera, frame_width, frame_height):
        """
        Initializes the camera based on the provided parameter.
        :param using_zed_camera: Boolean indicating whether to use the ZED camera.
        """
        self.using_zed_camera = using_zed_camera
        self.frame_width = frame_width
        self.frame_height = frame_height
        print("Initializing camera...")

        if self.using_zed_camera:
            self.initialize_zed_camera()
        else:
            self.initialize_standard_camera()

        print("Camera initialized")

    def initialize_zed_camera(self):
        global sl
        self.zed = sl.Camera()
        self.init = sl.InitParameters()
        self.init.camera_resolution = sl.RESOLUTION.HD1080  # Resolution setup (HD 1080p)
        self.init.depth_mode = sl.DEPTH_MODE.NONE  # No depth information
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 1)
        self.status = self.zed.open(self.init)
        if self.status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Error opening ZED camera: {self.status}")

    def initialize_standard_camera(self):
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Error opening the standard camera")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

    def get_frame(self):
        """
        Captures a frame from the camera.
        :return: The captured frame or None if an error occurs.
        """
        if self.using_zed_camera:
            return self.get_zed_frame()
        else:
            return self.get_standard_frame()

    def get_zed_frame(self):
        global sl
        if self.zed.grab() != sl.ERROR_CODE.SUCCESS:
            print("Error grabbing frame from ZED camera")
            return None
        else:
            image = sl.Mat()
            self.zed.retrieve_image(image, sl.VIEW.LEFT)
            frame = image.get_data()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)
            return frame

    def get_standard_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Error grabbing frame from standard camera")
            return None
        return frame

    def close(self):
        """
        Releases the camera resources.
        """
        if self.using_zed_camera:
            self.zed.close()
        else:
            self.cap.release()
        cv2.destroyAllWindows()


def load_calibration(file_path):
    fs = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)
    camera_matrix = fs.getNode("K").mat()
    dist_coeffs = fs.getNode("D").mat()
    fs.release()
    if camera_matrix is None or dist_coeffs is None:
        raise ValueError("Calibration file is missing required data")
    return camera_matrix, dist_coeffs

def detect_markers(frame, marker_queue, camera_matrix, dist_coeffs, drop_zone_id):
    """
    Detects ArUco markers and computes the camera's position relative to the markers.
    Displays information about detected markers.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)

    camera_relative_position = None  # Store camera position relative to the drop zone marker as tvec

    if ids is not None:
        frame = aruco.drawDetectedMarkers(frame, corners)

        for i, marker_id in enumerate(ids.flatten()):
            marker_queue.put(marker_id)
            if marker_id == drop_zone_id:
                corner = corners[i][0]
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners[i], MARKER_SIZE, camera_matrix, dist_coeffs)
                rvec, tvec = rvecs[0].flatten(), tvecs[0].flatten()

                # Store tvec for the drop zone marker
                camera_relative_position = (marker_id, tvec)
                #print(f"Marker ID: {marker_id}, rvec: {rvec}, tvec: {tvec}")
                
                # Draw axes and label the marker
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
                label_marker(frame, corner, marker_id, tvec)

    return camera_relative_position, frame
'''
def detect_markers(frame, marker_queue, camera_matrix, dist_coeffs, drop_zone_id):
    """
    Detects ArUco markers and computes the camera's position relative to the markers.
    Displays information about detected markers.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)

    camera_relative_position = None  # Store camera position relative to the drop zone marker as tvec

    if ids is not None:
        frame = aruco.drawDetectedMarkers(frame, corners)

        for i, marker_id in enumerate(ids.flatten()):
            marker_queue.put(marker_id)
            if marker_id == drop_zone_id:
                corner = corners[i][0]
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners[i], MARKER_SIZE, camera_matrix, dist_coeffs)
                
                # Ensure valid pose estimation
                if rvecs is not None and tvecs is not None:
                    rvec, tvec = rvecs[0].flatten(), tvecs[0].flatten()
                    
                    # Store tvec for the drop zone marker
                    camera_relative_position = (marker_id, tvec)
                    
                    # Print the pose for debugging purposes
                    print(f"Marker ID: {marker_id}, rvec: {rvec}, tvec: {tvec}")
                    
                    # Draw the axes (continuously in each frame where the marker is detected)
                    cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
                    label_marker(frame, corner, marker_id, tvec)
                else:
                    print(f"Pose estimation failed for marker {marker_id}")
    
    # Show frame with the drawn axes and markers
    return camera_relative_position, frame
'''
def display_camera_position(frame, camera_relative_position):
    """
    Display the camera's relative position to the drop zone marker in the corner of the CV2 window.
    """
    if camera_relative_position is not None:
        marker_id, tvec = camera_relative_position

        # Generate the distance info string
        side_distance = tvec[0]  # X-axis distance
        forward_distance = tvec[1]  # Y-axis distance
        height_distance = tvec[2]  # Z-axis distance
        distance_info = f"Marker {marker_id}: Fwd: {forward_distance:.2f} m, Side: {side_distance:.2f} m, Ht: {height_distance:.2f} m"

        # Display the string on the frame
        text_position = (10, 30)  # Top-left corner
        cv2.putText(frame, distance_info, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

def label_marker(frame, corner, marker_id, tvec):
    color = (0, 255, 0) if marker_id == 0 else (0, 0, 255)
    zone_label = "Drop Zone" if marker_id == 0 else "Non-Drop Zone"
    zone_label_position = np.mean(corner, axis=0).astype(int)
    
    cv2.polylines(frame, [corner.astype(int)], isClosed=True, color=color, thickness=3)
    cv2.putText(frame, zone_label, tuple(zone_label_position), cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 2, cv2.LINE_AA)
    cv2.putText(frame, f"ID: {marker_id}", (zone_label_position[0], zone_label_position[1] + 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 2, cv2.LINE_AA)
    cv2.putText(frame, f"Marker ID: {marker_id}, rvec: {RVEC}, tvec: {tvec}", (zone_label_position[0], zone_label_position[1] + 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 2, cv2.LINE_AA)

def get_image_dimensions_meters(dimensions, camera_matrix, frame_altitude):
    """
    Calculates the Ground Sampling Distance (GSD) in meters per pixel for the ZED camera, then calculates the
    width and height of the image in meters.

    Parameters:
    - dimensions: Tuple of (height, width) for the image in pixels.
    - camera_matrix: Camera matrix for the ZED camera.

    Returns:
    - Tuple of (width, height) for the image in meters.
    """

    STANDARD_SENSOR_SIZE = 35  # mm
    fov_vert = 70  # degrees
    fov_horizontal = 110  # degrees
    frame_width = dimensions[1]
    frame_height = dimensions[0]
    sensor_width = 4.8e-3  # sensor width in meters
    sensor_height = 3.6e-3  # focal length in meters
    sensor_size = sqrt(sensor_width ** 2 + sensor_height ** 2)

    focal_length = (camera_matrix[0][0] * sensor_width) / frame_width

    magnification = (STANDARD_SENSOR_SIZE / sensor_size) * (focal_length / 43.26661531)

    magnification = 0.45

    frame_height = frame_altitude * tan(radians(fov_vert / 2.0)) * 2.0 * magnification
    frame_width = frame_altitude * tan(radians(fov_horizontal / 2.0)) * 2.0 * magnification

    return frame_width, frame_height

'''
def get_detected_markers(frame, camera: Camera = None):
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    # Initialize the aruco dictionary and parameters
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)

    # Initialize the list of detected markers
    detected_markers = []

    # Detect the markers in the frame
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)  # , parameters=parameters)

    if ids is not None:
        # Show the detected markers
        frame = aruco.drawDetectedMarkers(frame, corners)

        for i, marker_id in enumerate(ids.flatten()):
            corner = corners[i][0]
            # Estimate pose for the current marker
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corner, MARKER_SIZE, camera_matrix, dist_coeffs)

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
            # Display rvec and tvec below the ID
   
        camera.showFrame(frame)

        # Add all detected ids to the list
        detected_markers.extend(ids)

    else:
        camera.getFrameAndShow()

    # Return the dictionary of detected markers
    print(f"Detected markers: {detected_markers}")
    return detected_markers
    '''


def flyInSearchPattern(vehicle: Vehicle, location_queue, isMarkerFound, distance_to_marker_queue):
    tvec = distance_to_marker_queue.get()
    tx = tvec[0]  # X-axis distance
    ty = tvec[1]  # Y-axis distance
    horizontal_distance = np.sqrt(tx**2 + ty**2)
    angle_xy = np.degrees(np.arctan2(ty, tx))
    approx_marker_coords = geodesic(meters=horizontal_distance).destination(
    Point(vehicle.location.global_relative_frame.lat, 
    vehicle.location.global_relative_frame.lon), angle_xy)
    markerLocation = LocationGlobalRelative(approx_marker_coords.latitude, approx_marker_coords.longitude, vehicle.location.global_relative_frame.alt)
    print(markerLocation)