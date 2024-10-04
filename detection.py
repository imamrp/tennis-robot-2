import cv2
import numpy as np
import tensorflow as tf

class TennisBallDetector:
    def __init__(self, model_path="tf_testing/detect.tflite", min_conf=0.8):
        self.min_conf = min_conf
        self.interpreter = tf.lite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -6)

    def process_frame(self, frame):
        input_shape = self.input_details[0]['shape']
        resized_frame = cv2.resize(frame, (input_shape[2], input_shape[1]))
        input_data = np.expand_dims(resized_frame, axis=0)

        if self.input_details[0]['dtype'] == np.float32:
            input_data = (np.float32(input_data) - 127.5) / 127.5

        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()

        boxes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]
        scores = self.interpreter.get_tensor(self.output_details[0]['index'])[0]

        detected_balls = []

        for i in range(len(scores)):
            if (scores[i] > self.min_conf) and (scores[i] <= 1.0):
                imH, imW, _ = frame.shape
                ymin = int(max(1, (boxes[i][0] * imH)))
                xmin = int(max(1, (boxes[i][1] * imW)))
                ymax = int(min(imH, (boxes[i][2] * imH)))
                xmax = int(min(imW, (boxes[i][3] * imW)))
                center_x = int((xmin + xmax) / 2)
                center_y = int((ymin + ymax) / 2)
                radius = int(max((xmax - xmin), (ymax - ymin)) / 2)

                detected_balls.append(((center_x, center_y), radius))

        detected_balls = sorted(detected_balls, key=lambda x: -x[1])

        if not detected_balls:
            # print("Processing in sections")
            detected_balls = self.process_frame_in_sections(frame)

        return detected_balls

    def process_frame_in_sections(self, frame):
        imH, imW, _ = frame.shape
        third_H = imH // 3
        third_W = imW // 3
        detected_balls = []

        for i in range(3):
            section = frame[third_H:third_H*2, i*third_W:(i+1)*third_W]
            input_shape = self.input_details[0]['shape']
            resized_section = cv2.resize(section, (input_shape[2], input_shape[1]))
            input_data = np.expand_dims(resized_section, axis=0)

            if self.input_details[0]['dtype'] == np.float32:
                input_data = (np.float32(input_data) - 127.5) / 127.5

            self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
            self.interpreter.invoke()

            boxes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]
            scores = self.interpreter.get_tensor(self.output_details[0]['index'])[0]

            for j in range(len(scores)):
                if (scores[j] > self.min_conf) and (scores[j] <= 1.0):
                    ymin = int(boxes[j][0] * input_shape[1])
                    xmin = int(boxes[j][1] * input_shape[2])
                    ymax = int(boxes[j][2] * input_shape[1])
                    xmax = int(boxes[j][3] * input_shape[2])

                    scaled_xmin = xmin * third_W / input_shape[2]
                    scaled_xmax = xmax * third_W / input_shape[2]
                    scaled_ymin = ymin * third_H / input_shape[1] + third_H
                    scaled_ymax = ymax * third_H / input_shape[1] + third_H

                    center_x = int((scaled_xmin + scaled_xmax) / 2) + i * third_W
                    center_y = int((scaled_ymin + scaled_ymax) / 2)
                    radius = int(max((scaled_xmax - scaled_xmin), (scaled_ymax - scaled_ymin)) / 2)

                    detected_balls.append(((center_x, center_y), radius))

        detected_balls = sorted(detected_balls, key=lambda x: -x[1])
        return detected_balls


    def get_circle_1_center(self, detected_balls):
        if len(detected_balls) > 0:
            return detected_balls[0][0]
        else:
            return None
    
    def get_circle_1_radius(self, detected_balls):
        if len(detected_balls) > 0:
            return detected_balls[0][1]
        else:
            return None

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()
