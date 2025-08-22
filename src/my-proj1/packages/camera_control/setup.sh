#!/bin/bash

# YOLO Setup Script for Raspberry Pi 3
# This script installs YOLO dependencies for the camera_control package

echo "Starting YOLO setup for Raspberry Pi 3..."

# Update system packages
echo "Updating system packages..."
sudo apt update
sudo apt upgrade -y

# Install Python and pip
echo "Installing Python and pip..."
sudo apt install python3-pip -y

# Upgrade pip to latest version
echo "Upgrading pip..."
pip install -U pip

# Install Ultralytics YOLO
echo "Installing Ultralytics YOLO..."
pip install ultralytics

echo "YOLO setup completed successfully!"
echo "You can now run: rosrun camera_control yolo_controller_node.py"