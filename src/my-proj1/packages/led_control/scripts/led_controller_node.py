#!/usr/bin/env python3

import rospy
from std_srvs.srv import SetBool, SetBoolRequest
import sys

class LEDController:
    def __init__(self):
        rospy.init_node('led_controller_node', anonymous=True)
        
        # Wait for the service to be available
        service_name = '/duckiealexa/led_emitter_node/set_pattern'
        rospy.loginfo(f"Waiting for service {service_name}")
        rospy.wait_for_service(service_name)
        
        # Create service proxy
        self.set_pattern_service = rospy.ServiceProxy(service_name, SetBool)
        rospy.loginfo("LED Controller Node initialized")
    
    def set_led_pattern(self, pattern_name):
        try:
            # Create service request
            request = SetBoolRequest()
            request.data = pattern_name
            
            # Call the service
            response = self.set_pattern_service(request)
            
            if response.success:
                rospy.loginfo(f"Successfully set LED pattern to: {pattern_name}")
            else:
                rospy.logwarn(f"Failed to set LED pattern: {response.message}")
            
            return response.success
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
    
    def run_interactive_mode(self):
        rospy.loginfo("LED Controller Interactive Mode")
        rospy.loginfo("Available colors: GREEN, BLUE, RED, WHITE")
        
        while not rospy.is_shutdown():
            try:
                color = input("Enter LED color (GREEN/BLUE/RED/WHITE) or 'quit' to exit: ").strip().upper()
                
                if color == 'QUIT':
                    break
                
                if color in ['GREEN', 'BLUE', 'RED', 'WHITE']:
                    self.set_led_pattern(color)
                else:
                    print("Invalid color. Please choose from: GREEN, BLUE, RED, WHITE")
                    
            except KeyboardInterrupt:
                break
        
        rospy.loginfo("LED Controller shutting down")

def main():
    try:
        controller = LEDController()
        controller.run_interactive_mode()
            
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()