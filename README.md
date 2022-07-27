# ERP42 Ros Driver

ROS package for controlling [WeGo-ERP42](https://wego-robotics.com/wego-erp42/) platform

## Installation

```
sudo apt install ros-{distro}-ackermann-msg
catkin_make
source devel/setup.bash

sudo chmod a+rx /dev/ttyUSB0

rosrun erp42_driver driver.py
```

## Controling ERP42

publish `AckermannDrive` msg to `"/ackermann_cmd"`

```
AckermannDrive
float32 steering_angle                      # desired steering angle in radians
float32 steering_angle_velocity             # unused
float32 speed                               # desired speed in m/s
float32 acceleration                        # set value between 0 ~ -1.0 to set brakes
float32 jerk                                # unused
```

## Reading ERP42 status


```
$ rostopic echo /erp42_status
mode: manual mode"
e stop: "E-STOP Off"
gear: "neutral"
speed: 0
steer: 0
brake: 0
alive: 12
enc: -17
---
mode: manual mode"
e stop: "E-STOP Off"
gear: "neutral"
speed: 0
steer: 0
brake: 0
alive: 13
enc: -17
---
mode: manual mode"
e stop: "E-STOP Off"
gear: "neutral"
speed: 0
steer: 0
brake: 0
alive: 14
enc: -17
---
```
