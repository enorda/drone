import cv2
import cv2.aruco as aruco

USING_ZED_CAMERA = True  # Set to True if using the ZED camera, False otherwise


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
        frame = aruco.drawDetectedMarkers(frame, corners, ids)
        camera.showFrame(frame)

        # Add all detected ids to the list
        detected_markers.extend(ids)

    else:
        camera.getFrameAndShow()

    # Return the dictionary of detected markers
    print(f"Detected markers: {detected_markers}")
    return detected_markers


if __name__ == "__main__":
    camera = Camera()
    while True:
        marker_list = get_detected_markers(camera.getFrame(), camera)
        if cv2.waitKey(1) == ord('q'):
            break
    camera.close()
