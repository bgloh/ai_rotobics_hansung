#!/usr/bin/env python3

import rospy
import os
import sys

class MotorController:
    def __init__(self):
        rospy.init_node('motor_controller_node', anonymous=True)
        rospy.loginfo("Motor Controller Node initialized")
    
    def send_motor_command(self, vel_left, vel_right):
        try:
            # Use rostopic pub command directly
            cmd = f'rostopic pub -1 /duckiealexa/wheels_driver_node/wheels_cmd duckietown_msgs/WheelsCmdStamped "{{header: {{stamp: now, frame_id: \"\"}}, vel_left: {vel_left}, vel_right: {vel_right}}}"'
            rospy.loginfo(f"Sending motor command: left={vel_left}, right={vel_right}")
            
            result = os.system(cmd)
            
            if result == 0:
                rospy.loginfo(f"Successfully sent motor command")
                return True
            else:
                rospy.logwarn(f"Failed to send motor command")
                return False
            
        except Exception as e:
            rospy.logerr(f"Motor command failed: {e}")
            return False
    
    def stop_motors(self):
        return self.send_motor_command(0.0, 0.0)
    
    def move_forward(self, speed=0.3):
        return self.send_motor_command(speed, speed)
    
    def move_backward(self, speed=0.3):
        return self.send_motor_command(-speed, -speed)
    
    def turn_left(self, speed=0.3):
        return self.send_motor_command(-speed, speed)
    
    def turn_right(self, speed=0.3):
        return self.send_motor_command(speed, -speed)
    
    def run_interactive_mode(self):
        rospy.loginfo("Motor Controller Interactive Mode")
        rospy.loginfo("Available commands: W=FORWARD, S=BACKWARD, A=LEFT, D=RIGHT, X=STOP, Q=QUIT")
        
        while not rospy.is_shutdown():
            try:
                char = input("Enter command (W/S/A/D/X/Q): ").strip().upper()
                
                if char == 'Q':
                    self.stop_motors()
                    break
                elif char == 'W':
                    self.move_forward()
                elif char == 'S':
                    self.move_backward()
                elif char == 'A':
                    self.turn_left()
                elif char == 'D':
                    self.turn_right()
                elif char == 'X':
                    self.stop_motors()
                else:
                    print("Invalid command. Please use: W=FORWARD, S=BACKWARD, A=LEFT, D=RIGHT, X=STOP, Q=QUIT")
                    
            except KeyboardInterrupt:
                self.stop_motors()
                break
        
        rospy.loginfo("Motor Controller shutting down")

def main():
    try:
        controller = MotorController()
        controller.run_interactive_mode()
            
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()