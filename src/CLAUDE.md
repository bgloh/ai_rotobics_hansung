# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS (Robot Operating System) workspace for Duckiebot robotics development, containing packages for controlling LED lights, motors, cameras, and high-level vehicle movement through interactive command-line interfaces.

## Architecture

The workspace follows standard ROS Catkin workspace structure:
- `my-proj1/packages/` - Contains ROS packages
  - `led_control` - Controls Duckiebot LED patterns via rosservice calls
  - `motor_control` - Low-level wheel control via direct motor commands
  - `car_control` - High-level vehicle control using Twist2DStamped messages
  - `camera_control` - Camera image processing with grayscale and edge detection
  - `my_first_package` - Basic publisher/subscriber example nodes

### Key Components

**LED Controller** (`led_control/scripts/led_controller_node.py`):
- Interactive single-character commands (G/B/R/W/O/Q)
- Uses `rosservice call /duckiealexa/led_emitter_node/set_pattern` 
- Supports GREEN, BLUE, RED, WHITE, LIGHT_OFF patterns

**Motor Controller** (`motor_control/scripts/motor_controller_node.py`):
- Low-level wheel control with WASD + X commands
- Direct ROS publisher using WheelsCmdStamped messages
- Publishes to `/duckiealexa/wheels_driver_node/wheels_cmd`
- Optimized for fast execution without subprocess calls

**Car Controllers** (`car_control/scripts/`):
- **car_controller_node.py**: Enter-based control with uppercase commands (W/S/A/D/Q/E/X/Z)
- **car_controller2_node.py**: Real-time control with lowercase commands (w/s/a/d/q/e/x/z)
- Uses Twist2DStamped messages for precise velocity control
- Publishes to `/{ROBOT_NAME}/car_cmd_switch_node/cmd`
- Supports combined movements (forward+turn) and ROS parameter configuration

**Camera Controller** (`camera_control/scripts/camera_controller_node.py`):
- Real-time image processing with grayscale conversion and edge detection
- Publishes processed images to separate topics with text overlays
- Docker-compatible (no cv2.imshow, republishes instead)
- Topics: `/{vehicle_name}/camera_node/image/processed/grayscaled/compressed` and `/edgedetected/compressed`

## Build and Run Commands

### Building the workspace:
```bash
cd /path/to/robotics_hansung/src
catkin_make
source devel/setup.bash
```

### Running nodes:
```bash
# LED controller (interactive mode)
rosrun led_control led_controller_node.py

# Low-level motor controller (wheel control)
rosrun motor_control motor_controller_node.py

# High-level car controller (Enter-based)
rosrun car_control car_controller_node.py

# High-level car controller (real-time, recommended)
rosrun car_control car_controller2_node.py

# Camera controller (image processing)
rosrun camera_control camera_controller_node.py

# Basic publisher/subscriber examples
rosrun my_first_package my_publisher_node.py
rosrun my_first_package my_subscriber_node.py
```

## Development Notes

- All nodes are Python 3 scripts with ROS dependencies
- LED controller uses `os.system()` for rosservice calls
- Motor controller uses direct ROS publishers with WheelsCmdStamped messages (optimized)
- Car controllers use Twist2DStamped for high-level movement control
- Camera controller is Docker-compatible, republishes instead of displaying images
- Interactive modes:
  - LED/Motor: Single character + Enter
  - car_controller_node.py: Uppercase + Enter
  - car_controller2_node.py: Lowercase, real-time (curses-based)
- All controllers support multi-threaded execution for responsiveness
- ROS parameters supported for speed configuration in car controllers