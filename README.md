# ROS - Interactive Rviz Interface for IK Computation with Capacity Margin Index

This repository includes nodes for generating and visualizing Cartesian Available : (1) Velocity Polytope (2) Force Polytope (3) Desired Polytope (4) Running Inverse Kinematics Computation for Serial Robots

## Installation
This repository was tested on UR5 robot with [UR5](https://github.com/KeerthiSagarSN/universal_robot.git) package. The robot is controllled through an external PC with [ROS Noetic](http://wiki.ros.org/noetic) and Ubuntu 20.04 LTS. 
### Hardware requirements
* External PC. Our specifications were;
```
ntel® Core™ i7-10700K CPU @ 3.80GHz × 16
RAM: 16 GB
Graphics: NVIDIA Corporation GP106GL [Quadro P2200]
```
* UR5 Robot


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

