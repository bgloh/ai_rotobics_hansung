#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def publisher_node():
    # Initialize the ROS node
    rospy.init_node('my_publisher_node', anonymous=True)
    
    # Create a publisher object
    pub = rospy.Publisher('chatter', String, queue_size=10)
    
    # Set the publishing rate (10 Hz)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        # Create and publish a message
        hello_str = "Hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass