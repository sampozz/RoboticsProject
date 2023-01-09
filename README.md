# Robotics Project - Group V

This repository contains the source files for the project of the course Introduction to Robotics at the University of Trento.

# Getting started 

The Locosim framework is required to run the project: https://github.com/mfocchi/locosim  

Clone this repository in your home folder, then cd into `ros_ws` and build the project by running the following commands:

```bash
$ catkin_make install
$ source devel/setup.bash
```

Install requirements for YOLOv5 image recognition algorithm:

```bash
$ cd ros_ws/src/robotic_vision/scripts/yolov5
$ pip install -r requirements.txt
```

To start the simulation environment (RVIZ + Gazebo) use Python:

```bash
$ python3 locosim/ur5_generic.py
```

Run the ROS nodes and start the simulation using:

```bash
$ roslaunch main_controller main_sim.launch
```
