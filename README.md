# ERP42 ROS Driver

ROS package for controlling [WeGo-ERP42](https://wego-robotics.com/wego-erp42/) platform

## Installation

```
sudo apt install ros-{distro}-ackermann-msg
catkin_make
source devel/setup.bash

sudo chmod a+rw /dev/ttyUSB0

rosrun erp42_driver driver.py
```

## Quick Start

publish `AckermannDrive` msg to `"/ackermann_cmd"` to control vehicle

```
AckermannDrive
float32 steering_angle                      # desired steering angle in radians
float32 steering_angle_velocity             # unused
float32 speed                               # desired speed in m/s
float32 acceleration                        # set value between 0 ~ -1.0 to set brakes
float32 jerk                                # unused
```

Currently only publishes diagnostic messages
- mode: `manual mode` or `auto mode`
- e_stop: `E-STOP On` or `E-stop Off`
- gear: one of `forward drive`, `netural`, `backward drive`
- speed: speed in kph
- steer: steering angle in degrees
- brake: raw brake value 1: 0%, 200: 100% range: 0 ~ 200
- alive: value incremented to check connection health
- enc: wheel encoder count, counts rotations

publishes DiagnosticStatus msg as `/erp42_status`

sample:

```
level: 0
name: ''
message: ''
hardware_id: ''
values:
  -
    key: "auto_mode"
    value: "manual mode"
  -
    key: "e_stop"
    value: "E-STOP Off"
  -
    key: "gear"
    value: "forward drive"
  -
    key: "speed"
    value: "0.0"
  -
    key: "steer"
    value: "0"
  -
    key: "brake"
    value: "0"
  -
    key: "enc"
    value: "0"
  -
    key: "alive"
    value: "0"
  -
    key: "raw"
    value: ""
  -
    key: "valid"
    value: "False"
```
