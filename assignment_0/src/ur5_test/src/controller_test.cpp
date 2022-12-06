#include "ros/ros.h"
#include "ur5_controller_lib/ur5_controller.h"
#include "kinematics_lib/ur5_kinematics.h"
#include <iostream>

int main(int argc, char **argv)
{
    // ROS Node initialization
    ros::init(argc, argv, "ur5_controller");

    // Istantiate controller
    UR5Controller controller(1000.0, 0.005, 10.0);

    // Wait a bit, just to avoid shit
    ros::Duration(0.5).sleep();

    Coordinates dest;
    RotationMatrix rot = RotationMatrix::Identity();
    dest << 0.4, -0.4, 0.4;
    euler_to_rot(0, 0, 0, rot);

    controller.ur5_follow_path(dest, rot, 50);

    return 0;
}