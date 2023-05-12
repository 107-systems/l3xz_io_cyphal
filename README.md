<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `ros2_cyphal_bridge`
==================================
[![Build Status](https://github.com/107-systems/ros2_cyphal_bridge/actions/workflows/ros2.yml/badge.svg)](https://github.com/107-systems/ros2_cyphal_bridge/actions/workflows/ros2.yml)
[![Spell Check status](https://github.com/107-systems/ros2_cyphal_bridge/actions/workflows/spell-check.yml/badge.svg)](https://github.com/107-systems/ros2_cyphal_bridge/actions/workflows/spell-check.yml)

This package provides the interface between ROS and [L3X-Z](https://github.com/107-systems/l3xz)'s [Cyphal](https://opencyphal.org) connected components.

<p align="center">
  <a href="https://github.com/107-systems/l3xz"><img src="https://raw.githubusercontent.com/107-systems/.github/main/logo/l3xz-logo-memento-mori-github.png" width="40%"></a>
</p>

#### How-to-build
```bash
cd $COLCON_WS/src
git clone --recursive https://github.com/107-systems/ros2_cyphal_bridge
cd $COLCON_WS
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_cyphal_bridge
```

#### How-to-run
```bash
cd $COLCON_WS
. install/setup.bash
ros2 launch ros2_cyphal_bridge bridge.py
```

#### Interface Documentation
Published Topics
|               Default name               |                                             Type                              | Description                                             |
|:----------------------------------------:|:-----------------------------------------------------------------------------:|---------------------------------------------------------|
|     `/l3xz/ros2_cyphal_bridge/heartbeat` |  [`std_msgs/UInt64`](https://docs.ros2.org/foxy/api/std_msgs/msg/UInt64.html) | Heartbeat signal containing the node uptime in seconds. |

##### Parameters
| Name | Default | Description |
|:-:|:-:|-|
| `can_iface` | `can0` | Network name of CAN bus. |
| 'can_node_id' | 100 | Cyphal/CAN node id. |

#### Notes
Configure light mode from bash:
```bash
ros2 topic pub --once /l3xz/light_mode/target std_msgs/msg/Int8 "{data: 3}"
```
Turn on Zubax Orel 20 from bash:
```bash
# yakut
y pub --period=0.1 5001:reg.udral.service.common.Readiness.0.1 "{value: 3}"
y pub --period=0.1 5002:reg.udral.service.actuator.common.sp.Scalar.0.1 "{value: 10.0}"
# ros
ros2 topic pub -r 10 /l3xz/pump/readiness/target std_msgs/msg/Int8 "{data: 3}"
ros2 topic pub -r 10 /l3xz/pump/rpm/target std_msgs/msg/Float32 "{data: 10}"
```
