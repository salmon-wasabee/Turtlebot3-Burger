import torch
import os
from super_gradients.training import models
from super_gradients.common.object_names import Models
from ultralytics import YOLO
import cv2
import math 
import numpy as np

# models
pose_model = models.get("yolo_nas_pose_l", pretrained_weights="coco_pose")
device = 'cuda' if torch.cuda.is_available() else 'cpu'
pose_model.to(device)

def process_frame(frame):
   frame_copy = frame.copy()
   results = pose_model(frame_copy)
   # coordinates
   for r in results:
       boxes = r.boxes
       for box in boxes:
           # bounding box
           x1, y1, x2, y2 = box.xyxy[0]
           x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int valuesput box in cam
           # pose estimation
           roi = frame[y1:y2, x1:x2]
           confidence = 0.6
           pose_results = pose_model.predict(roi, conf=confidence)

           # visualize the pose estimation results
           for result in pose_results:
               keypoints = result['keypoints']
               for keypoint in keypoints:
                  x, y = keypoint
                  cv2.circle(frame, (x, y), radius=3, color=(0, 255, 0), thickness=-1)

   return frame

