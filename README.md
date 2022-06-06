<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: L3X-Z
===================
L3X-Z base robot control package (ROS).

### Related Repositories
| Name | Description |
|-|-|
| [l3xz](https://github.com/107-systems/l3xz) | ROS based control software for L3X-Z, a mixed electric/hydraulic hexapod. |
| [l3xz_teleop](https://github.com/107-systems/l3xz_teleop) | Teleoperation for L3X-Z via PS3 joystick and ROS topics. |
| [l3xz-mapping](https://github.com/107-systems/l3xz-mapping) | Docker container containing the mapping stack for L3X-Z. |
| [l3xz-hw](https://github.com/107-systems/l3xz-hw) | L3X-Z Hexapod hardware design files (3D model, printed parts, etc.) |
| [l3xz-hw_leg-controller](https://github.com/107-systems/l3xz-hw_leg-controller) | L3X-Z Hexapod leg controller hardware design files |
| [l3xz-fw_aux-controller](https://github.com/107-systems/l3xz-fw_aux-controller) | Firmware for the auxiliary controller (alarm LEDs and emergency stop) |
| [l3xz-fw_leg-controller](https://github.com/107-systems/l3xz-fw_leg-controller) | Firmware for the leg controller. |
| [l3xz-fw_radiation_sensor](https://github.com/107-systems/l3xz-fw_radiation_sensor) | Firmware for the radiation sensor |

### Developer Setup
#### Target (Robot)
[Raspberry Pi 4/8 GB](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/), [Buster](https://www.raspberrypi.com/software/operating-systems/#raspberry-pi-os-legacy), [ROS Noetic Ninjemys](https://varhowto.com/install-ros-noetic-raspberry-pi-4/).
#### Host (Devlopment PC)
Ubuntu 20.04 LTS, [ROS Noetic Ninjemys](http://wiki.ros.org/noetic/Installation/Ubuntu).

### How-to-build
```bash

# Clone this repository into catkin_ws/src.
git clone https://github.com/107-systems/l3xz
# Invoke catkin_make from the catkin workspace root.
source /opt/ros/noetic/setup.bash
catkin_make
```

### How-to-run
```bash
source devel/setup.bash
roslaunch l3xz robot.launch
```
