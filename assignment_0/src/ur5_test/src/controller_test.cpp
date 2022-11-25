#include "ros/ros.h"
#include "ur5_controller_lib/ur5_controller.h"
#include <iostream>

int main(int argc, char** argv)
{
    // ROS Node initialization 
    ros::init(argc, argv, "ur5_controller");

    // Istantiate controller
    UR5Controller controller;

    // Define a destination for the robot
    Coordinates dest;
    dest << 0.1978, -0.6579, 0.2324;
    RotationMatrix rot;
    rot << 0.2990, -0.8513, 0.4321, -0.6107, 0.1766, 0.7720, -0.7333, -0.4941, -0.4670;

    // Wait a bit, just to avoid shit
    ros::Duration(0.5).sleep();

    // Move!
    controller.ur5_move_to(dest, rot);
        
    return 0;
}