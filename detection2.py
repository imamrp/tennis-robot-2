import cv2
import os
import numpy as np
from copy import deepcopy
from ultralytics import YOLO
from ultralytics.utils import ops


class TennisBallDetector:
    def __init__(self, model_path = "tf_testing/best.pt"):
        self.model = YOLO(model_path)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -11)

    def process_frame(self, img, sections = False):
        """
        function:
            detect target(s) in an image
        input:
            img: image, e.g., image read by the cv2.imread() function
        output:
            bboxes: list of lists, box info [label,[x,y,width,height]] for all detected targets in image
            img_out: image with bounding boxes and class labels drawn on
        """
        bboxes = self._get_bounding_boxes(img)
        detected_balls = []
        # draw bounding boxes on the image
        for bbox in bboxes:
            #  translate bounding box info back to the format of [x1,y1,x2,y2]
            center_x = bboxes[0]
            center_y = bboxes[1]
            radius = int(max((bboxes[2]), (bboxes[3])))

            if radius < 100 and radius > 1: # TODO retest radius
                detected_balls.append(((center_x, center_y), radius))
        detected_balls = sorted(detected_balls, key=lambda x: -x[1])

        return detected_balls

    def _get_bounding_boxes(self, cv_img):
        """
        function:
            get bounding box and class label of target(s) in an image as detected by YOLOv8
        input:
            cv_img    : image, e.g., image read by the cv2.imread() function
            model_path: str, e.g., 'yolov8n.pt', trained YOLOv8 model
        output:
            bounding_boxes: list of lists, box info [label,[x,y,width,height]] for all detected targets in image
        """

        # predict target type and bounding box with your trained YOLO

        predictions = self.model.predict(cv_img, imgsz=640, verbose=False)

        # get bounding box and class label for target(s) detected
        bounding_boxes = []
        for prediction in predictions:
            boxes = prediction.boxes
            for box in boxes:
                # bounding format in [x, y, width, height]
                box_cord = box.xywh[0]
                bounding_boxes.append(box_cord)

        return bounding_boxes

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
