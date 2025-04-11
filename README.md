# PRIZM ROS2
Unofficial ROS2 Jazzy compaitibility for Tetrix PRIZM controller

## Setup

Clone this repository with the following line:
``` bash
git clone https://github.com/LTU-Actor/prizm_ros2.git
```
Build the package with the following lines:
``` bash
colcon build --packages-select prizm_ros2
```
``` bash
source <ros2 workspace>/install/setup.bash
```


## Usage

Launch the program with the following line:
``` bash
ros2 launch prizm_ros2 launch.py serial_port:=<path to PRIZM controller>
```

## ROS Info
### Nodes
- prizm_node: Handles ROS2 to PRIZM serial communication.

### Topics
Name | Message Type | Description
-----|--------------|--------------
twist_controller | geometry_msgs/Twist | Controls robot movement.
green_led | std_msgs/Bool | Controls the state of the green LED.
red_led | std_msgs/Bool | Controls the state of the red LED.

### Parameters
- serial_port: Specifies the serial device path to the PRIZM controller (default: /dev/ttyUSB0).
