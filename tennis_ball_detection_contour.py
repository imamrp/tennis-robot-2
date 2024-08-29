import cv2
import numpy as np

class TennisBallDetector:
    def __init__(self, lower_color=np.array([30, 40, 40]), upper_color=np.array([90, 255, 255])):
        self.lower_color = lower_color
        self.upper_color = upper_color
        self.cap = cv2.VideoCapture(1)
        self.kernel = np.ones((5, 5), np.uint8)
        self.se21 = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 21))
        self.se11 = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))
        self.min_contour_area = 100
        self.min_radius = 5

    def process_frame(self, frame):
        # Convert the frame to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create a mask for the color of the tennis ball
        mask = cv2.inRange(hsv, self.lower_color, self.upper_color)

        # Apply morphological operations to clean up the mask
        mask = cv2.erode(mask, self.kernel, iterations=2)
        mask = cv2.dilate(mask, self.kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.se21)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.se11)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # List to hold detected circles and their radii
        detected_balls = []

        # Draw circles around the detected contours
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_contour_area:
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)

                if radius > self.min_radius:  # Filter out very small circles
                    detected_balls.append((center, radius))

        # Sort detected balls by radius (largest to smallest)
        detected_balls = sorted(detected_balls, key=lambda x: -x[1])

        return detected_balls

    def get_circle_1_center(self, detected_balls):
        if len(detected_balls) > 0:
            return detected_balls[0][0]  # Return the center of the first circle
        else:
            return None

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main():
    detector = TennisBallDetector()

    while True:
        # Capture frame-by-frame
        ret, frame = detector.cap.read()
        if not ret:
            break

        # Process the frame to find detected balls
        detected_balls = detector.process_frame(frame)

        # Get the center of the first detected ball, if any
        center = detector.get_circle_1_center(detected_balls)
        if center:
            print(f"Center of the circle labelled 1: {center}")
        else:
            print("No circle labelled 1 detected.")

        # Create a copy of the frame to draw the circles
        circle_frame = frame.copy()

        # Draw and label the circles
        for i, (center, radius) in enumerate(detected_balls):
            label = str(i + 1)
            cv2.circle(circle_frame, center, radius, (0, 255, 0), 2)
            cv2.putText(circle_frame, label, (center[0] - 10, center[1] + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Display the frames
        cv2.imshow('Raw Webcam Frame', frame)
        cv2.imshow('Tennis Ball Detection', circle_frame)

        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    detector.release()

if __name__ == "__main__":
    main()
