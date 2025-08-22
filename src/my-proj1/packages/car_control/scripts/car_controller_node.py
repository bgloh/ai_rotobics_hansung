#!/usr/bin/env python3

import os
import rospy
import sys
import threading
from duckietown_msgs.msg import Twist2DStamped

class CarController:
    def __init__(self):
        rospy.init_node('car_controller_node', anonymous=True)
        self.current_command = None
        self.command_lock = threading.Lock()
        
        # Get robot name from environment variable
        self.robot_name = os.environ.get('VEHICLE_NAME', 'duckiealexa')
        
        # Create publisher for car commands
        self.cmd_topic = f"/{self.robot_name}/car_cmd_switch_node/cmd"
        self.pub = rospy.Publisher(self.cmd_topic, Twist2DStamped, queue_size=1)
        
        rospy.loginfo(f"Car Controller Node initialized for robot: {self.robot_name}")
        rospy.loginfo(f"Publishing to topic: {self.cmd_topic}")
    
    def send_car_command(self, v, omega):
        """Send car movement command using Twist2DStamped message
        
        Args:
            v (float): Linear velocity in m/s (forward is positive)
            omega (float): Angular velocity in rad/s (counter-clockwise is positive)
        """
        try:
            # Create Twist2DStamped message
            msg = Twist2DStamped()
            msg.header.stamp = rospy.Time.now()
            msg.v = v
            msg.omega = omega
            
            self.pub.publish(msg)
            return True
            
        except Exception as e:
            rospy.logerr(f"Car command failed: {e}")
            return False
    
    def stop_car(self):
        """Stop the car"""
        return self.send_car_command(0.0, 0.0)
    
    def move_forward(self, speed=0.3):
        """Move forward at specified speed"""
        return self.send_car_command(speed, 0.0)
    
    def move_backward(self, speed=0.3):
        """Move backward at specified speed"""
        return self.send_car_command(-speed, 0.0)
    
    def turn_left(self, angular_speed=2.0):
        """Turn left with specified angular speed"""
        return self.send_car_command(0.0, angular_speed)
    
    def turn_right(self, angular_speed=2.0):
        """Turn right with specified angular speed"""
        return self.send_car_command(0.0, -angular_speed)
    
    def move_forward_left(self, linear_speed=0.3, angular_speed=1.0):
        """Move forward while turning left"""
        return self.send_car_command(linear_speed, angular_speed)
    
    def move_forward_right(self, linear_speed=0.3, angular_speed=1.0):
        """Move forward while turning right"""
        return self.send_car_command(linear_speed, -angular_speed)
    
    def execute_command(self, command):
        """Execute movement command based on user input"""
        with self.command_lock:
            if command == 'W':
                self.move_forward()
                rospy.loginfo("Moving forward (v=0.3, ω=0.0)")
            elif command == 'S':
                self.move_backward()
                rospy.loginfo("Moving backward (v=-0.3, ω=0.0)")
            elif command == 'A':
                self.turn_left()
                rospy.loginfo("Turning left (v=0.0, ω=2.0)")
            elif command == 'D':
                self.turn_right()
                rospy.loginfo("Turning right (v=0.0, ω=-2.0)")
            elif command == 'Q':
                self.move_forward_left()
                rospy.loginfo("Forward + Left (v=0.3, ω=1.0)")
            elif command == 'E':
                self.move_forward_right()
                rospy.loginfo("Forward + Right (v=0.3, ω=-1.0)")
            elif command == 'X':
                self.stop_car()
                rospy.loginfo("Stopping (v=0.0, ω=0.0)")
    
    def run_interactive_mode(self):
        """Run interactive car control mode"""
        rospy.loginfo("Car Controller Interactive Mode")
        rospy.loginfo("Available commands:")
        rospy.loginfo("  W = Forward")
        rospy.loginfo("  S = Backward") 
        rospy.loginfo("  A = Turn Left")
        rospy.loginfo("  D = Turn Right")
        rospy.loginfo("  Q = Forward + Left")
        rospy.loginfo("  E = Forward + Right")
        rospy.loginfo("  X = Stop")
        rospy.loginfo("  Z = Quit")
        rospy.loginfo("Commands use Twist2DStamped messages for precise control")
        
        while not rospy.is_shutdown():
            try:
                char = input("Enter command (W/S/A/D/Q/E/X/Z): ").strip().upper()
                
                if char == 'Z':
                    self.stop_car()
                    break
                elif char in ['W', 'S', 'A', 'D', 'Q', 'E', 'X']:
                    # Execute command immediately in separate thread for faster response
                    threading.Thread(target=self.execute_command, args=(char,), daemon=True).start()
                else:
                    print("Invalid command. Please use: W/S/A/D/Q/E/X/Z")
                    
            except KeyboardInterrupt:
                self.stop_car()
                break
        
        rospy.loginfo("Car Controller shutting down")

def main():
    try:
        controller = CarController()
        controller.run_interactive_mode()
            
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()