import cv2
import os
import pyzed as sl
from scout.camera_process import Camera;

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
