<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: L3X-Z
===================
L3X-Z base robot control package (ROS).

#### MikroTik "Base Station"
Bridge-IP: 192.168.88.2 (station bridge, nv2, pre-shared-key, l...).
#### MikroTik "Robot"
Bridge-IP: 192.168.88.1 (bridge, nv2, pre-shared-key, l...)
#### Robot Rasperry Pi
IP: 192.168.88.5
#### Control PC
IP: 192.168.88.3

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
roslaunch l3xz l3xz.launch
```
