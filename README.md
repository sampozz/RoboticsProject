# Robotics Project - Group V

This repository contains the source files for the project of the course Introduction to Robotics at the University of Trento.

# Getting started 

The Locosim framework is required to run the project: https://github.com/mfocchi/locosim  

Clone this repository in your home folder, then cd into `ros_ws` and build the project by running the following commands:

```bash
$ catkin_make install
$ source devel/setup.bash
```

To start the simulation environment (RVIZ + Gazebo) use Python:

```bash
$ python3 locosim/ur5_generic.py
```

To run the project, run the main controller using:

```bash
$ roslaunch main_controller main_sim.launch
```

# Catkin Packages

Below is a brief explanation of the catkin packages created for this project. Please, refer to the PDF report for a more detailed description. 

## Main Controller

This package contains a C++ source that implements three different finite state machine to control the entire procedure for the three project assignments. The state functions are differentiated using namespaces. The Main Controller communicates with the other controllers using ROS Service calls.

## UR5 Controller

This package contains a C++ class `UR5Controller` which implements the algorithms for controlling the UR5 manipulator. The ROS Node `ur5_controller_node` instantiates an `UR5Controller` object and receives ROS Service calls. 

## Shelfino Controller

This package contains a C++ class `ShelfinoController` which implements the algorithms for controlling the Shelfino mobile robot. The ROS Node `shelfino_controller_node` instantiates a `ShelfinoController` object and receives ROS Service calls. 

## Kinematics Library

This package is library of C++ functions which implement the kinematics primitives for controlling the UR5 and Shelfino robots. 

## Robotic Vision

This package contains the impletation of YOLOv5 used for computer vision and detecting the megablocks on the field. The C++ ROS Node `vision_node` subscribes to YOLO topic and buffers detections for the Main Controller.

## Shelfino Gazebo

This package is used to simulate Shelfino mobile robot on Gazebo and creates its ROS Topics.

## Gazebo ROS Link Attacher

This package is used to create a dynamic link into Gazebo between the UR5 gripper and the megablock to simulate the grasping. 