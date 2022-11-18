# Robotics Project - Group V

## Instructions 

The Locosim framework is required to run the project: https://github.com/mfocchi/locosim  

Run `python3 ur5_generic.py` to start the simulation environment (RVIZ + Gazebo).

Modify `params.py` to set initial parameters.

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

## Assignment 0

This is a Catkin Workspace intended for testing features. 