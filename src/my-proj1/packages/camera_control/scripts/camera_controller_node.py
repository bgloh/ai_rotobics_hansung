#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge

class CameraReaderNode(DTROS):

   def __init__(self, node_name):
       super(CameraReaderNode, self).__init__(
           node_name=node_name,
           node_type=NodeType.VISUALIZATION
       )
       self._vehicle_name = os.environ['VEHICLE_NAME']
       self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
       self._grayscale_topic = f"/{self._vehicle_name}/camera_node/image/processed/grayscaled/compressed"
       self._edge_topic = f"/{self._vehicle_name}/camera_node/image/processed/edgedetected/compressed"
       self._bridge = CvBridge()
       
       # Subscriber for incoming camera images
       self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
       
       # Publishers for processed images
       self.gray_pub = rospy.Publisher(self._grayscale_topic, CompressedImage, queue_size=1)
       self.edge_pub = rospy.Publisher(self._edge_topic, CompressedImage, queue_size=1)

   def add_text_overlay(self, image, text):
       """Add text overlay to image"""
       height, width = image.shape[:2]
       font = cv2.FONT_HERSHEY_SIMPLEX
       font_scale = 0.7
       color = (255, 255, 255)  # White color
       thickness = 2
       
       # Get text size to position it properly
       text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
       text_x = width - text_size[0] - 10  # 10 pixels from right edge
       text_y = height - 20  # 20 pixels from bottom
       
       cv2.putText(image, text, (text_x, text_y), font, font_scale, color, thickness)
       return image

   def callback(self, msg):
       # Convert compressed image to OpenCV format
       image = self._bridge.compressed_imgmsg_to_cv2(msg)
       
       # Process 1: Grayscale
       gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
       gray_bgr = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)  # Convert to 3-channel
       gray_with_text = self.add_text_overlay(gray_bgr.copy(), "PROCESSED:GRAYSCALE")
       
       # Process 2: Edge detection
       edges = cv2.Canny(gray_image, 50, 150)
       edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)  # Convert to 3-channel
       edges_with_text = self.add_text_overlay(edges_bgr.copy(), "PROCESSED:EDGE-DETECTED")
       
       # Convert to compressed messages
       gray_msg = self._bridge.cv2_to_compressed_imgmsg(gray_with_text)
       gray_msg.header = msg.header
       
       edge_msg = self._bridge.cv2_to_compressed_imgmsg(edges_with_text)
       edge_msg.header = msg.header
       
       # Publish both processed images
       self.gray_pub.publish(gray_msg)
       self.edge_pub.publish(edge_msg)
       
       rospy.loginfo("Processed and published grayscale and edge-detected images")

if __name__ == '__main__':
   node = CameraReaderNode(node_name='camera_reader_node')
   rospy.spin()