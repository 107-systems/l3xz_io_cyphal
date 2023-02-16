<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `l3xz_ros_cyphal_bridge`
======================================
[![Build Status](https://github.com/107-systems/l3xz_ros_cyphal_bridge/actions/workflows/ros2.yml/badge.svg)](https://github.com/107-systems/l3xz_ros_cyphal_bridge/actions/workflows/ros2.yml)
[![Spell Check status](https://github.com/107-systems/l3xz_ros_cyphal_bridge/actions/workflows/spell-check.yml/badge.svg)](https://github.com/107-systems/l3xz_ros_cyphal_bridge/actions/workflows/spell-check.yml)

This package provides the interface between ROS and [L3X-Z](https://github.com/107-systems/l3xz)'s [Cyphal](https://opencyphal.org) connected components.

<p align="center">
  <a href="https://github.com/107-systems/l3xz"><img src="https://raw.githubusercontent.com/107-systems/.github/main/logo/l3xz-logo-memento-mori-github.png" width="40%"></a>
</p>

#### How-to-build
```bash
colcon_ws/src$ git clone --recursive https://github.com/107-systems/l3xz_ros_cyphal_bridge
colcon_ws$ source /opt/ros/galactic/setup.bash
colcon_ws$ colcon build --packages-select l3xz_ros_cyphal_bridge
```

#### How-to-run
```bash
colcon_ws$ source install/setup.bash
colcon_ws$ ros2 launch l3xz_ros_cyphal_bridge bridge.py
```

##### Parameters
| Name | Default | Description |
|:-:|:-:|-|
| `can_iface` | `can0` | Network name of CAN bus. |
