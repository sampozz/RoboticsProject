#include "ros/ros.h"
#include "ur5_controller_lib/ur5_controller.h"
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
    dest << 0.3, 0.3, -0.3;
    RotationMatrix rot = RotationMatrix::Identity();

    double linear_space[10];
    for (int i = 0; i < 10; i++)
    {
        linear_space[i] = (double)i * 5. / 10.;
    }

    auto xe = [](double t) {
        Coordinates c;
        c << 0.4 * sin(2 * M_PI * t), 0.4 * cos(2 * M_PI * t), 0.5;
        // c << 0.4 * t, 0.4 * (1 - t), 0.5;
        return c;
    };


    for (int i = 0; i < 10; i++)
    {
        Coordinates x = xe(linear_space[i]);
        controller.ur5_move_to(x, rot);
        if (i % 2 == 0) {
            controller.ur5_set_gripper(50);
        } else {
            controller.ur5_set_gripper(100);
        }
    }

    return 0;
}