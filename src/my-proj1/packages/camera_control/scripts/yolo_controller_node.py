#!/usr/bin/env python3

import os
import rospy
import numpy as np
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False
    rospy.logwarn("Ultralytics not available. Install with: pip install ultralytics")

class YOLOControllerNode(DTROS):

    def __init__(self, node_name):
        super(YOLOControllerNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._yolo_topic = f"/{self._vehicle_name}/camera_node/image/processed/yolo/compressed"
        self._bridge = CvBridge()
        
        # Load YOLO model
        self.load_yolo_model()
        
        # Subscriber for incoming camera images
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        
        # Publisher for YOLO processed images
        self.yolo_pub = rospy.Publisher(self._yolo_topic, CompressedImage, queue_size=1)
        
        rospy.loginfo("YOLO Controller Node initialized")

    def load_yolo_model(self):
        """Load YOLOv8n (nano) model for object detection"""
        try:
            if ULTRALYTICS_AVAILABLE:
                # Load YOLOv8 nano model (smallest, fastest for Pi 3)
                self.model = YOLO('yolov8n.pt')
                rospy.loginfo("YOLOv8n model loaded successfully")
                self.yolo_enabled = True
            else:
                rospy.logwarn("Ultralytics not available. YOLO detection disabled.")
                self.model = None
                self.yolo_enabled = False
                
        except Exception as e:
            rospy.logerr(f"Failed to load YOLO model: {e}")
            self.model = None
            self.yolo_enabled = False

    def detect_objects(self, image):
        """Perform object detection on image using YOLOv8n"""
        if not self.yolo_enabled or self.model is None:
            return self.fallback_detection(image)
        
        try:
            # Run inference with YOLOv8
            results = self.model(image, conf=0.25, iou=0.45, verbose=False)
            
            detections = []
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # Extract box coordinates, confidence, and class
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        conf = box.conf[0].cpu().numpy()
                        cls = int(box.cls[0].cpu().numpy())
                        
                        # Convert to [x, y, w, h] format
                        x, y, w, h = int(x1), int(y1), int(x2-x1), int(y2-y1)
                        
                        detections.append({
                            'bbox': [x, y, w, h],
                            'confidence': float(conf),
                            'class_id': cls,
                            'class_name': self.model.names[cls]
                        })
            
            return detections
            
        except Exception as e:
            rospy.logerr(f"YOLO detection failed: {e}")
            return []

    def fallback_detection(self, image):
        """Fallback detection method when YOLO is not available"""
        # Simple demo: detect large contours as "objects"
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:  # Filter small contours
                x, y, w, h = cv2.boundingRect(contour)
                detections.append({
                    'bbox': [x, y, w, h],
                    'confidence': 0.5,
                    'class_id': 0,
                    'class_name': 'object'
                })
        
        return detections

    def draw_detections(self, image, detections):
        """Draw bounding boxes and labels on image"""
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        for detection in detections:
            x, y, w, h = detection['bbox']
            confidence = detection['confidence']
            class_name = detection['class_name']
            
            # Generate color based on class
            color = tuple(map(int, np.random.uniform(0, 255, 3)))
            
            # Draw bounding box
            cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
            
            # Draw label with confidence
            label_text = f"{class_name}: {confidence:.2f}"
            label_size = cv2.getTextSize(label_text, font, 0.5, 1)[0]
            
            # Background rectangle for text
            cv2.rectangle(image, (x, y - label_size[1] - 10), 
                         (x + label_size[0], y), color, -1)
            
            # Text
            cv2.putText(image, label_text, (x, y - 5), font, 0.5, (255, 255, 255), 1)
        
        # Add "PROCESSED:YOLO" text
        height, width = image.shape[:2]
        text = "PROCESSED:YOLO"
        text_size = cv2.getTextSize(text, font, 0.7, 2)[0]
        text_x = width - text_size[0] - 10
        text_y = height - 20
        cv2.putText(image, text, (text_x, text_y), font, 0.7, (255, 255, 255), 2)
        
        return image

    def callback(self, msg):
        """Process incoming camera image for object detection"""
        try:
            # Convert compressed image to OpenCV format
            image = self._bridge.compressed_imgmsg_to_cv2(msg)
            
            # Perform object detection
            detections = self.detect_objects(image)
            
            # Draw detections on image
            result_image = self.draw_detections(image.copy(), detections)
            
            # Convert back to compressed message
            yolo_msg = self._bridge.cv2_to_compressed_imgmsg(result_image)
            yolo_msg.header = msg.header
            
            # Publish YOLO processed image
            self.yolo_pub.publish(yolo_msg)
            
            detection_count = len(detections)
            if detection_count > 0:
                class_names = [d['class_name'] for d in detections]
                rospy.loginfo(f"YOLO detected {detection_count} objects: {', '.join(set(class_names))}")
            else:
                rospy.loginfo("YOLO processed image - no objects detected")
            
        except Exception as e:
            rospy.logerr(f"YOLO processing failed: {e}")

if __name__ == '__main__':
    node = YOLOControllerNode(node_name='yolo_controller_node')
    rospy.spin()