import cv2
import numpy as np
import time
import os

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
        import pyzed.sl as sl
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

def take_picture(frame, filename):
    """
    Saves the captured frame as a picture.
    :param frame: The frame to save.
    :param filename: The filename to save the image as.
    """
    cv2.imwrite(filename, frame)
    print(f"Picture saved as {filename}")

def main():
    # Set camera resolution
    frame_width = 1920
    frame_height = 1080
    camera = Camera(using_zed_camera=True, frame_width=frame_width, frame_height=frame_height)

    # Create a directory to save pictures
    save_dir = "captured_images"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    picture_counter = 0

    while True:
        frame = camera.get_frame()
        if frame is None:
            print("Failed to capture frame")
            break

        # Display the frame
        cv2.imshow("Camera Feed", frame)

        # Wait for user input
        key = cv2.waitKey(1) & 0xFF

        # Take a picture when the user presses the 't' key
        if key == ord('t'):  # 't' for take picture
            picture_filename = os.path.join(save_dir, f"image_{picture_counter}.jpg")
            take_picture(frame, picture_filename)
            picture_counter += 1

        # Exit when the user presses the 'q' key
        elif key == ord('q'):  # 'q' to quit
            break

    camera.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
