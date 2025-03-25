import cv2

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Change backend if needed

if not cap.isOpened():
    print("Camera failed to open.")
    exit()

while True:
    ret, frame = cap.read()  # Read frame
    if not ret:
        print("Failed to grab frame.")
        break

    cv2.imshow("Camera Feed", frame)  # Show frame

    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
        break

cap.release()
cv2.destroyAllWindows()
