<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `l3xz_io`
=======================
L3X-Z base robot control IO package (ROS).

### How-to-build
```bash
# Clone this repository into colcon_ws/src.
git clone https://github.com/107-systems/l3xz_io
# Invoke 'colcon build' from repository root.
source /opt/ros/galactic/setup.bash
colcon build --packages-select l3xz_io
```

### How-to-run
```bash
source install/setup.bash
ros2 launch l3xz robot.py
```
