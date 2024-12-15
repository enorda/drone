import cv2
import cv2.aruco as aruco
import numpy as np

USING_ZED_CAMERA = False  # Set to True if using the ZED camera, False otherwise


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
            self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
            if not self.cap.isOpened():
                print("Error opening the camera")
                return None

        cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
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

def load_calibration(file_path):
    """
    Load the camera matrix and distortion coefficients from a YAML file.
    """
    fs = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)
    camera_matrix = fs.getNode("K").mat()
    dist_coeffs = fs.getNode("D").mat()
    fs.release()
    return camera_matrix, dist_coeffs

def get_detected_markers(frame, camera: Camera = None):
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    # Initialize the aruco dictionary and parameters
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)

    # Load calibration data
    calibration_file_path = "calibration_chessboard.yaml"  # Path to your calibration file
    camera_matrix, dist_coeffs = load_calibration(calibration_file_path)

    # Marker size in meters (adjust to match your marker's actual size)
    marker_size = 0.1

    # Initialize the list of detected markers
    detected_markers = []

    # Detect the markers in the frame
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)

    if ids is not None:
        # Show the detected markers
        frame = aruco.drawDetectedMarkers(frame, corners)

        for i, marker_id in enumerate(ids.flatten()):
            corner = corners[i][0]

            # Estimate pose for each marker
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners[i], marker_size, camera_matrix, dist_coeffs)

            # Draw the axis for the marker
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], 0.05)  # Axis length = 0.05 meters

            # Extract pose information
            rvec = rvecs[0].flatten()
            tvec = tvecs[0].flatten()

            # Marker info (position and orientation)
            print(f"Marker ID: {marker_id}")
            print(f"Translation Vector (tvec): {tvec}")
            print(f"Rotation Vector (rvec): {rvec}")

            # Display marker information on the frame
            color = (0, 255, 0) if marker_id == 0 else (0, 0, 255)
            zone_label = "Drop Zone" if marker_id == 0 else "Non-Drop Zone"
            cv2.polylines(frame, [corner.astype(int)], isClosed=True, color=color, thickness=3)

            zone_label_position = np.mean(corner, axis=0).astype(int)
            cv2.putText(frame, zone_label, tuple(zone_label_position), cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 2, cv2.LINE_AA)
            id_label_position = (zone_label_position[0], zone_label_position[1] + 50)
            cv2.putText(frame, f"ID: {marker_id}", id_label_position, cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 2, cv2.LINE_AA)

        camera.showFrame(frame)
        detected_markers.extend(ids)

    else:
        camera.getFrameAndShow()

    print(f"Detected markers: {detected_markers}")
    return detected_markers


if __name__ == "__main__":
    camera = Camera()
    frame_width = int(camera.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(camera.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(camera.cap.get(cv2.CAP_PROP_FPS))

    # Initialize VideoWriter
    fps = 30
    output_filename = "output.avi"
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')  # Codec for AVI file
    video_writer = cv2.VideoWriter(output_filename, fourcc, fps, (frame_width, frame_height))

    while True:
        frame = camera.getFrame()
        if frame is None:
            break
        frame = cv2.resize(frame, (frame_width, frame_height))  # Ensure size consistency
        marker_list = get_detected_markers(frame, camera)
        video_writer.write(frame)  # Write the frame to the output file
        if cv2.waitKey(1) == ord('q'):
            break

    camera.close()
    video_writer.release()  # Ensure the video file is properly closed
    print(f"Video saved to {output_filename}")
