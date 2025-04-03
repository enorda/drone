import cv2
import cv2.aruco as aruco
import numpy as np
import pyzed as sl

from scout.camera_process import Camera

USING_ZED_CAMERA = False  # Set to True if using the ZED camera, False otherwise
FRAME_HEIGHT = 1280
FRAME_WIDTH = 720
FPS = 30
CALIBRATION_FILE_PATH = "calibration_chessboard.yaml"  # Path to your calibration file
MARKER_SIZE = 0.1  # Marker size in meters


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
    camera_matrix, dist_coeffs = load_calibration(CALIBRATION_FILE_PATH)
    camera = Camera(USING_ZED_CAMERA, FRAME_WIDTH, FRAME_HEIGHT)

    output_filename = "images/output.avi"

    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    video_writer = cv2.VideoWriter(output_filename, fourcc, FPS, (FRAME_WIDTH, FRAME_HEIGHT))

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