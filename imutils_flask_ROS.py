import torch
import torchvision
import cv2
import mediapipe as mp
import numpy as np
from ultralytics import YOLO
import os
import time
import requests
from imutils.video import VideoStream
import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge
import threading


weights_path = os.getcwd() + '/weights' + '/best.pt'  # Replace with your model weights file path
model = YOLO(weights_path)

yolo_conf_threshold = 0.5  # Detection confidence threshold

cv2_font = cv2.FONT_HERSHEY_SIMPLEX
cv2_font_scale = 1
cv2_font_color = (0, 0, 0)  # Black color
cv2_line_type = 2

## Mediapipe setup
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

mp_conf_threshold = 0.5 # Detection confidence threshold
mp_tracking_threshold = 0.5 # Tracking confidence threshold
image_captured = False
last_capture_time = time.time()

current_frame = None  # Global variable to store the most recent video frame
cv_bridge = CvBridge()  # Create a CvBridge instance

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage, '/i069/camera/color/image_raw/compressed', self.listener_callback, 30)
        

    def listener_callback(self, data):
        global current_frame
        #self.get_logger().info('Receiving video frame')
        current_frame = data.data  # Store the received Image message directly

def is_keypoints_present(mp_results, frame, xmin, ymin, xmax, ymax):
    ## Get height and width
    image_height, image_width, _ = frame.shape

    ## Get keypoints coordinates
    if results.pose_landmarks is not None:
        landmarks = results.pose_landmarks.landmark
        nose = [landmarks[mp_pose.PoseLandmark.NOSE.value].x * image_width, landmarks[mp_pose.PoseLandmark.NOSE.value].y * image_height]
        elbow_l = [landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].x * image_width, landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].y * image_height]
        elbow_r = [landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].x * image_width, landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].y * image_height]
        knee_l = [landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].x * image_width, landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].y * image_height]
        knee_r = [landmarks[mp_pose.PoseLandmark.RIGHT_KNEE.value].x * image_width, landmarks[mp_pose.PoseLandmark.RIGHT_KNEE.value].y * image_height]

        keypoints = [nose, elbow_l, elbow_r, knee_l, knee_r]
        for x, y in keypoints:
            if not (xmin <= x <= xmax and ymin <= y <= ymax):
                return False
            return True
    
    return False 

## Edit Video Stream Source
cap = VideoStream(src=0).start()

## Setup YOLO & Mediapipe instance

## Edit Video Stream Source
def run_inf():
## Setup YOLO & Mediapipe instance
    global current_frame
    global results
    global last_capture_time
    while True:
        if current_frame is None:
            break
        nparr = np.frombuffer(current_frame, np.uint8)
        current_cv_frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        current_cv_frame = cv2.resize(current_cv_frame,(0,0),fx=0.5,fy=0.5)
        #current_cv_frame = cv2.cvtColor(current_cv_frame, cv2.COLOR_RGB2BGR)


        ## Get height and width
        image_height, image_width, _ = current_cv_frame.shape

        # run the YOLO model on the frame
        detections = model(current_cv_frame)[0]

        # loop over the detections
        for data in detections.boxes.data.tolist():
            # extract the confidence (i.e., probability) associated with the detection
            confidence = data[4]
            class_id = detections.names[data[5]]
            annotated_frame = detections[0].plot()

            if float(confidence) > yolo_conf_threshold:
            
                xmin, ymin, xmax, ymax = int(data[0]), int(data[1]), int(data[2]), int(data[3])
                results = mp_pose.Pose(min_detection_confidence=mp_conf_threshold, min_tracking_confidence=mp_tracking_threshold).process(current_cv_frame)

                # if the confidence is greater than the minimum confidence and all keypoints present
                # draw the bounding box on the frame

                if results.pose_landmarks is not None and is_keypoints_present(results.pose_landmarks.landmark,current_cv_frame, xmin, ymin, xmax, ymax):
                    mp_drawing.draw_landmarks(annotated_frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                    mp_drawing.DrawingSpec(color=(255,255,255), thickness=2, circle_radius=2), 
                    mp_drawing.DrawingSpec(color=(225,255,0), thickness=1, circle_radius=2) 
                     )
                    if True:
                        _, frame_data = cv2.imencode(".jpg", annotated_frame,[cv2.IMWRITE_JPEG_QUALITY, 50])
                        url = "http://localhost:8000/upload"
                        response = requests.post(url, files={"image": ("frame.jpg", frame_data.tobytes(), "image/jpeg")})
                        image_captured = True

                if time.time() - last_capture_time >= 5:
                    image_captured = False
                    last_capture_time = time.time()  

def main():
    rclpy.init()
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
     
if __name__ == "__main__":  
    ros_thread = threading.Thread(target=main)
    ros_thread.start()
    time.sleep(5)
    run_inf()     

