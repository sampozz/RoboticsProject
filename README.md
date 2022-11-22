# Robotics Project - Group V

This repository contains the source files for the project of the course Introduction to Robotics at the University of Trento.

# Getting started 

The Locosim framework is required to run the project: https://github.com/mfocchi/locosim  

Run `python3 locosim/ur5_generic.py` to start the simulation environment (RVIZ + Gazebo).

The `locosim` folder contains the sources that have been modified from the original locosim.

Modify `locosim/params.py` to set initial parameters.

# Develop and build assignments

The following rules apply to every assignment:
- Assignments are developed in different Catkin workspaces 
- Add the following instruction to `~/.bashrc` (replacing N with the assignment number):
  ```bash
    source $HOME/robotics_group_v/assignment_N/devel/setup.bash
  ```
- To build the project, cd to assignment root directory and run `catkin_make install`
- The `main_controller` package contains the launch file to start the required nodes
- Every node is developed in a different package inside the workspace
    - To create a new package use `catkin_create_pkg <package_name> <depends>`
- Python scripts are located in the `scripts` folder inside a package
- C++ sources are located in the `src` folder inside a package

# Assignment 0

This is a Catkin Workspace intended for testing features. 

## Package: Main controller

At the moment, this package is used only to launch the nodes using a launch file: `roslaunch main_controller main.launch`

Create a new launch file to test other nodes.

## Package: UR5 Controller 

`ur5_controller` package contains python scripts and C++ code for testing the ur5 manipulator kinematics. This package contains the following nodes:

- `scripts/sin_movement.py` and `src/sin_movement.cpp` are used to make the robot joints move following a sine wave. The two nodes do the same thing.  
- `src/direct_kinematics_test.cpp` is used to move the robot by changing the joints values q_des0 to different known values q_des1. A filter is applied to make the movement smooth.   
- `scripts/homing_procedure.py` is used to set robot joints to the initial value of params.py
 
## Package: View Camera Output

This package is used for debugging purposes, just to see the image from the camera point of view using opencv.

Launch with `roslaunch main_controller camera.launch`

## Package: Darknet ROS YOLOv3

This package comes from: https://github.com/leggedrobotics/darknet_ros

To use this nodes, change desired configuration in `darknet_ros/config/ros.yaml` and launch with `roslaunch darknet_ros yolo_v3.launch` 

We need to train the model with the Megablocks and replace the weights.
