# ROS - Interactive Rviz Interface for IK Computation with Capacity Margin Index

This repository includes a GUI panel to interactively select a 3D position for computing Inverse kinematics for a serial manipulator.

## Installation
This repository was tested with [UR5](https://github.com/KeerthiSagarSN/universal_robot.git) URDF package and Sawyer Robot URDF package. 
### Hardware requirements
* External PC. Our specifications were;
```
Intel® Core™ i7-10700K CPU @ 3.80GHz × 16
RAM: 16 GB
Graphics: NVIDIA Corporation GP106GL [Quadro P2200]
```


### Software and Library Requirements - Dependencies

* Ubuntu 20.04 LTS
* ROS Noetic
* Python3.x

* [PyQT5](https://pypi.org/project/PyQt5/) Library for GUI panels
```
$ pip install PyQt5
```
* [Interactive Markers](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Getting%20Started) 3D interactive Markers similar to Moveit End-effector marker
* [ROS-Visualization-Interactive-Marker](https://github.com/ros-visualization/interactive_markers.git) Interactive Markers for ROS Visualiation with servers and Prebuilt markers
* Python native libraries [Scipy](https://scipy.org/), [Numpy](https://numpy.org/)

## Launch interactive panel

```
$ roslaunch inverse_kinematics_interactive_rviz inverse_kinematics_interactive_rviz.launch
```

