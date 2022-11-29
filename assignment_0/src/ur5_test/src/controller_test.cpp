#include "ros/ros.h"
#include "ur5_controller_lib/ur5_controller.h"
#include <iostream>

int main(int argc, char **argv)
{
    // ROS Node initialization
    ros::init(argc, argv, "ur5_controller");

    // Istantiate controller
    UR5Controller controller;

    // Wait a bit, just to avoid shit
    ros::Duration(0.5).sleep();

    Coordinates dest;
    RotationMatrix rot = RotationMatrix::Identity();

    double linear_space[100];
    for (int i = 0; i < 100; i++)
    {
        linear_space[i] = (double)i * 5. / 100.;
    }

    auto xe = [](double t) {
        Coordinates c;
        c << 0.4 * sin(2 * M_PI * t), 0.4 * cos(2 * M_PI * t), 0.5;
        return c;
    };


    for (int i = 0; i < 100; i++)
    {
        Coordinates x = xe(linear_space[i]);
        controller.ur5_move_to(x, rot);
        
    }

    return 0;
}