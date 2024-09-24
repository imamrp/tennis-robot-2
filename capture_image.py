import cv2

# Open the camera (0 is usually the USB camera)
cap = cv2.VideoCapture(0)  # Use 0 if your USB camera is detected as /dev/video0

# Check if the camera opened successfully
if not cap.isOpened():
    print("Failed to open the camera")
else:
    # Capture a frame
    ret, frame = cap.read()
    if ret:
        # Save the captured image
        cv2.imwrite('image.jpg', frame)
        print("Image saved as image.jpg")
    else:
        print("Failed to capture image")

# Release the camera
cap.release()
