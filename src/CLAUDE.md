# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS (Robot Operating System) workspace for Duckiebot robotics development, containing packages for controlling LED lights and motors through interactive command-line interfaces.

## Architecture

The workspace follows standard ROS Catkin workspace structure:
- `my-proj1/packages/` - Contains ROS packages
  - `led_control` - Controls Duckiebot LED patterns via rosservice calls
  - `motor_control` - Controls Duckiebot movement via wheel commands
  - `my_first_package` - Basic publisher/subscriber example nodes

### Key Components

**LED Controller** (`led_control/scripts/led_controller_node.py`):
- Interactive single-character commands (G/B/R/W/O/Q)
- Uses `rosservice call /duckiealexa/led_emitter_node/set_pattern` 
- Supports GREEN, BLUE, RED, WHITE, LIGHT_OFF patterns

**Motor Controller** (`motor_control/scripts/motor_controller_node.py`):
- WASD + X controls for movement (W=forward, S=backward, A=left, D=right, X=stop)
- Uses `rostopic pub` to `/duckiealexa/wheels_driver_node/wheels_cmd`
- Multi-threaded for responsive control

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

# Motor controller (interactive mode) 
rosrun motor_control motor_controller_node.py

# Basic publisher/subscriber examples
rosrun my_first_package my_publisher_node.py
rosrun my_first_package my_subscriber_node.py
```

## Development Notes

- All nodes are Python 3 scripts with ROS dependencies
- LED controller uses `os.system()` for rosservice calls
- Motor controller uses `subprocess.Popen()` for faster rostopic publishing
- Interactive modes expect single character input followed by Enter
- Motor commands execute immediately in separate threads for responsiveness