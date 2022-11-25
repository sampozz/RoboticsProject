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
    dest << 0.2, -0.2, 0.2;
    RotationMatrix rot;
    // rot << 0.2990, -0.8513, 0.4321, -0.6107, 0.1766, 0.7720, -0.7333, -0.4941, -0.4670;
    euler_to_rot(0, M_PI / 2, M_PI / 2, rot);

    // Wait a bit, just to avoid shit
    ros::Duration(0.5).sleep();

    // Move!
    controller.ur5_move_to(dest, rot);
        
    return 0;
}