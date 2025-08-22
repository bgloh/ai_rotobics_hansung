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
       self._processed_topic = f"/{self._vehicle_name}/camera_node/image/processed/compressed"
       self._bridge = CvBridge()
       
       # Subscriber for incoming camera images
       self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
       
       # Publisher for processed images
       self.pub = rospy.Publisher(self._processed_topic, CompressedImage, queue_size=1)

   def callback(self, msg):
       # Convert compressed image to OpenCV format
       image = self._bridge.compressed_imgmsg_to_cv2(msg)
       
       # Process image: convert to grayscale
       gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
       # Convert back to 3-channel for compressed image message compatibility
       processed_image = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
       
       # Convert back to compressed image message
       processed_msg = self._bridge.cv2_to_compressed_imgmsg(processed_image)
       processed_msg.header = msg.header  # Preserve original header
       
       # Publish processed image
       self.pub.publish(processed_msg)
       
       rospy.loginfo("Processed and published grayscale camera image")

if __name__ == '__main__':
   node = CameraReaderNode(node_name='camera_reader_node')
   rospy.spin()