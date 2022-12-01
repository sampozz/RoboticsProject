# Robotics Project - Group V

This repository contains the source files for the project of the course Introduction to Robotics at the University of Trento.

# Getting started 

The Locosim framework is required to run the project: https://github.com/mfocchi/locosim  

Run `python3 locosim/ur5_generic.py` to start the simulation environment (RVIZ + Gazebo).

The `locosim` folder contains the sources that have been modified from the original locosim.

Modify `locosim/params.py` to set initial parameters.

To run a node use `rosrun <pkg name> <node name>` or create a .launch file and use `roslaunch <pkg name> <launchfile name>`.

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

This package will contain a C++ source that implements a finite state machine to control everything.

At the moment, this package is used only to launch the nodes using a launch file: `roslaunch main_controller main.launch`

Create a new launch file to test other nodes.

## Package: UR5 kinematics lib

This is a library which exports the ur5 kinematics functions `ur5_direct()`, `ur5_inverse()` and `euler_to_rot()`.

## Package: UR5 controller lib

This is the main package that implements the ur5 movement. This is a library which exports a class `UR5Controller`.

The controller class defines a `ur5_move_to()` function which is used to move the robot from the current position to the cartesian position passed as parameter.

## Package: UR5 test

`ur5_test` package contains python scripts and C++ code for testing the ur5 manipulator kinematics. This package contains the following nodes:

- `src/controller_test.cpp` just a template to test ur5 movement.
- `src/manual_control.cpp` used to manually set a position to the ur5 robot and get the relative information.

The following nodes work only if the gripper is set to `False` in `locosim/params.py`:

- `scripts/sin_movement.py` and `src/sin_movement.cpp` are used to make the robot joints move following a sine wave. The two nodes do the same thing.  
- `src/direct_kinematics_test.cpp` listen to topic /ur5/joint_states, get the values of the joints, compute direct kinematics and print cartesian coordinates relative to the current position of the robot.
- `src/inverse_kinematics_test.cpp` starting from the homing position, compute the desired joints from a hardcoded position and move the robot to that position.
- `scripts/homing_procedure.py` is used to set robot joints to the initial value of params.py
- `src/joint_movement.cpp` move the robot from the homing position to a hardcoded position specified by joint values.

## Package: Shelfino Gazebo

Start Shelfino in gazebo simulation environment, using `roslaunch shelfino_gazebo shelfino.launch`. 
 
## Package: View Camera Output

This package is used for debugging purposes, just to see the image from the camera point of view using opencv.

Launch with `roslaunch main_controller camera.launch`


