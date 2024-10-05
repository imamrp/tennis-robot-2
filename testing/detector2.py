import cv2
import supervision as sv
from inference import get_model

while True:
    # define the image url to use for inference
    image = cv2.VideoCapture(0)
    image.set(cv2.CAP_PROP_EXPOSURE, -4)
    
    # load a pre-trained yolov8n model
    model = get_model(model_id="tennis-ball-model-izpej-90bri")
    
    # run inference on our chosen image, image can be a url, a numpy array, a PIL image, etc.
    results = model.infer(image)[0]
    
    # load the results into the supervision Detections api
    detections = sv.Detections.from_inference(results)
    
    # create supervision annotators
    bounding_box_annotator = sv.BoundingBoxAnnotator()
    label_annotator = sv.LabelAnnotator()
    
    # annotate the image with our inference results
    annotated_image = bounding_box_annotator.annotate(
        scene=image, detections=detections)
    annotated_image = label_annotator.annotate(
        scene=annotated_image, detections=detections)
    
    # display the image
    sv.plot_image(annotated_image)
