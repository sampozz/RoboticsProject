# Robotics Project - Group V

This repository contains the source files for the project of the course Fundamentals of Robotics at the University of Trento.  

## Project Report

The document [project_report](./project_report.pdf) contains a long and exhaustive report including the complete description of the code architecture, the UR5 and Shelfino motion planning strategies, the robotic vision algorithm and the outcomes of the project.  

## Authors

Samuele Pozzani, Enea Strambini, Daniel Marcon, Giacomo Tezza.  

# Demo

The following videos on YouTube show the results of the project on the simulation environment. The measured KPIs and a final analysis are available in the project report. All the three assignments have been completed!

- [Assignment 1](https://www.youtube.com/watch?v=aQaSSeZ6_o4)
- [Assignment 2](https://www.youtube.com/watch?v=nNXjxYvLdJU)
- [Assignment 3](https://www.youtube.com/watch?v=10f5vGYs3hg)

During the tests in the laboratory, the UR5 robot was recorded while performing the finite state machine related to assignment 2.  

- [UR5 test](https://www.youtube.com/watch?v=gDq3QjKvoGI)

# Getting started 

The [Locosim framework](https://github.com/mfocchi/locosim) is required to run the project.

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
$ roslaunch main_controller simulation.launch
```

# Acknowledgments

<a href="https://www.unitn.it/"><img src="./docs/unitn-logo.jpg" width="300px"></a>

## Doxygen docs

To generate Doxygen documentation, install Doxygen and execute:  

```bash
$ doxygen Doxyfile
``` 

HTML docs will be generated inside the `docs` folder.

## Copyright

MIT Licence or otherwise specified. See [license](./LICENSE.txt) file for details.
