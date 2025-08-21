#!/usr/bin/env python3

import rospy
import os
import sys

class LEDController:
    def __init__(self):
        rospy.init_node('led_controller_node', anonymous=True)
        rospy.loginfo("LED Controller Node initialized")
    
    def set_led_pattern(self, pattern_name):
        try:
            # Use rosservice call command directly
            cmd = f'rosservice call /duckiealexa/led_emitter_node/set_pattern "pattern_name: {{data: {pattern_name}}}"'
            rospy.loginfo(f"Calling service: {cmd}")
            
            result = os.system(cmd)
            
            if result == 0:
                rospy.loginfo(f"Successfully set LED pattern to: {pattern_name}")
                return True
            else:
                rospy.logwarn(f"Failed to set LED pattern to: {pattern_name}")
                return False
            
        except Exception as e:
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