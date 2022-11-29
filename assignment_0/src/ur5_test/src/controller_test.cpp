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
    dest << 0.4, -0.4, 0.6;
    RotationMatrix rot;
    euler_to_rot(-M_PI_4, M_PI_2, 0, rot);

    // Wait a bit, just to avoid shit
    ros::Duration(0.5).sleep();

    // Move!
    controller.ur5_move_to(dest, rot);
        
    return 0;
}