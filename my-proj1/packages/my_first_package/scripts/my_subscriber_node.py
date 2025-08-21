#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def subscriber_node():
    # Initialize the ROS node
    rospy.init_node('my_subscriber_node', anonymous=True)
    
    # Create a subscriber object
    rospy.Subscriber('chatter', String, callback)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    subscriber_node()