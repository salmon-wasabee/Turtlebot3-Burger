from ultralytics import YOLO
import cv2
import math 
import numpy as np

# model
model = YOLO("yolov8n-pose.pt")

# object classes
classNames = ["person"]

def process_frame(frame):
    frame_copy = frame.copy()
    results = model(frame_copy, classes=[0], stream=True)
    # coordinates
    for r in results:
        boxes = r.boxes
        for box in boxes:
            # bounding box
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int valuesput box in cam
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 3)

            # confidence
            confidence = math.ceil((box.conf[0]*100))/100
            print("Confidence --->",confidence)

            # class name
            cls = int(box.cls[0])
            print("Class name -->", classNames[cls])

            # object details
            org = [x1, y1]
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (255, 0, 0)
            thickness = 2
            cv2.putText(frame, classNames[cls], org, font, fontScale, color, thickness)
    return frame

