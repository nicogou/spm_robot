# Spherical Parallel Manipulator robot

## Introduction

This repository holds a ROS2 package for a spherical parallel manipulator robot.

## Teleoperation

In order to run the robot, build the package with colcon, source the install folder, and launch:
```shell
ros2 launch spm_robot spm_control.launch.py
```
This launches `ros2_control` with mock Hardware Interfaces and a Forward Command controller, as well as RViz with a model of the robot.

In another terminal run the teleoperation node:
```shell
ros2 run spm_robot spm_teleop
```

You can now control the platform's orientation by pressing the numbers keys. In addition to that there are three modes available:
- `s` - Single Axis Rotation mode: in a given orientation, the platform rotates around its own Z-axis
- `z` - Global Z Rotation: the platform keeps its orientation and rotates around the global Z-axis
- `c` - Cone mode: in a given orientation, the platform's normal vector describes a cone around the global Z-axis.
