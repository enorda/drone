import cv2
import cv2.aruco as aruco
import numpy as np

USING_ZED_CAMERA = False  # Set to True if using the ZED camera, False otherwise
CALIBRATION_FILE_PATH = "calibration_chessboard.yaml"  # Path to your calibration file
MARKER_SIZE = 0.1  # Marker size in meters


class Camera:
    def __init__(self):
        print("Initializing camera...")

        if USING_ZED_CAMERA:
            self.initialize_zed_camera()
        else:
            self.initialize_standard_camera()

        print("Camera initialized")

    def initialize_zed_camera(self):
        global sl
        import pyzed.sl as sl
        self.zed = sl.Camera()
        self.init = sl.InitParameters()
        self.init.camera_resolution = sl.RESOLUTION.HD1080
        self.init.depth_mode = sl.DEPTH_MODE.NONE
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 1)
        self.status = self.zed.open(self.init)
        if self.status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Error opening ZED camera: {self.status}")

    def initialize_standard_camera(self):
        self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            raise RuntimeError("Error opening the standard camera")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    def get_frame(self):
        if USING_ZED_CAMERA:
            return self.get_zed_frame()
        else:
            return self.get_standard_frame()

    def get_zed_frame(self):
        global sl
        if self.zed.grab() != sl.ERROR_CODE.SUCCESS:
            print("Error grabbing frame from ZED camera")
            return None
        image = sl.Mat()
        self.zed.retrieve_image(image, sl.VIEW.LEFT)
        frame = image.get_data()
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)

    def get_standard_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Error grabbing frame from standard camera")
            return None
        return frame

    def close(self):
        if USING_ZED_CAMERA:
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


def detect_markers(frame, camera_matrix, dist_coeffs):
    """
    Detects ArUco markers and computes the camera's position relative to the markers.
    Displays information about detected markers.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)

    camera_relative_positions = []  # Store camera position relative to markers for display

    if ids is not None:
        frame = aruco.drawDetectedMarkers(frame, corners)

        for i, marker_id in enumerate(ids.flatten()):
            corner = corners[i][0]
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners[i], MARKER_SIZE, camera_matrix, dist_coeffs)
            rvec, tvec = rvecs[0].flatten(), tvecs[0].flatten()

            # Calculate relative distances
            
            side_distance = tvec[0]  # X-axis distance
            forward_distance = tvec[1]  # Y-axis distance
            height_distance = tvec[2]  # Z-axis distance
            distance_info = f"Fwd: {forward_distance:.2f} m, Side: {side_distance:.2f} m, Ht: {height_distance:.2f} m"

            # Add to display list
            camera_relative_positions.append(distance_info)

            # Draw axes and label the marker
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
            label_marker(frame, corner, marker_id, tvec)

    return camera_relative_positions, frame

def display_camera_positions(frame, positions):
    """
    Display the camera's relative positions to detected markers in the corner of the CV2 window.
    """
    for i, position in enumerate(positions):
        text_position = (10, 30 + i * 30)  # Top-left corner, with vertical spacing for each marker
        cv2.putText(frame, position, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)


def label_marker(frame, corner, marker_id, tvec):
    color = (0, 255, 0) if marker_id == 0 else (0, 0, 255)
    zone_label = "Drop Zone" if marker_id == 0 else "Non-Drop Zone"
    zone_label_position = np.mean(corner, axis=0).astype(int)

    cv2.polylines(frame, [corner.astype(int)], isClosed=True, color=color, thickness=3)
    cv2.putText(frame, zone_label, tuple(zone_label_position), cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 2, cv2.LINE_AA)
    cv2.putText(frame, f"ID: {marker_id}", (zone_label_position[0], zone_label_position[1] + 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 2, cv2.LINE_AA)



def main():
    camera = Camera()
    camera_matrix, dist_coeffs = load_calibration(CALIBRATION_FILE_PATH)
    frame_width = 1280
    frame_height = 720
    fps = 30
    output_filename = "output.avi"

    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    video_writer = cv2.VideoWriter(output_filename, fourcc, fps, (frame_width, frame_height))

    try:
        while True:
            frame = camera.get_frame()
            if frame is None:
                break

            # Detect markers and get camera positions
            camera_positions, processed_frame = detect_markers(frame, camera_matrix, dist_coeffs)

            # Display camera positions in the corner of the window
            display_camera_positions(processed_frame, camera_positions)

            # Write frame to video and show it
            video_writer.write(processed_frame)
            cv2.imshow("Camera Feed", processed_frame)

            if cv2.waitKey(1) == ord('q'):
                break

    finally:
        camera.close()
        video_writer.release()
        print(f"Video saved to {output_filename}")


if __name__ == "__main__":
    main()