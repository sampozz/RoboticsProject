#include "ros/ros.h"
#include "ur5_controller_lib/ur5_controller.h"
#include "ur5_kinematics_lib/ur5_kinematics.h"
#include <iostream>

int main(int argc, char **argv)
{
    // ROS Node initialization
    ros::init(argc, argv, "ur5_controller");

    // Istantiate controller
    UR5Controller controller(1000.0, 0.005, 10.0);

    // Wait a bit, just to avoid shit
    ros::Duration(0.5).sleep();



    Coordinates initial_pos;
    initial_pos << 0.0232723, -0.253316, 0.478839;
    RotationMatrix initial_rot = RotationMatrix::Identity();
    euler_to_rot(0, 0, 0, initial_rot);
    Coordinates final_pos;
    final_pos << 0.02, -0.4, 0.479;
    RotationMatrix final_rot = RotationMatrix::Identity();
    // euler_to_rot(M_PI / 6.0, M_PI / 3.0, M_PI / 4.0, final_rot);


    double *theta = ur5_motion_plan(initial_pos, initial_rot, final_pos, final_rot, 0.01);

    JointStateVector desired_joints;
    for (int i = 0; i < 100; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            std::cout << theta[i*6 + j] << " ";
            desired_joints(j) = theta[i*6 + j];
        }
        std::cout << std::endl;
        if (desired_joints.norm() != 0)
            controller.send_joint_state(desired_joints);
    }







    return 0;
    /*
    Coordinates initial_pos;
    RotationMatrix initial_rot;
    JointStateVector current_joints;
    controller.ur5_get_joint_states(current_joints);
    ur5_direct(current_joints, initial_pos, initial_rot);

    Coordinates final_pos;
    final_pos << 0.02, -0.4, 0.479;
    RotationMatrix final_rot = RotationMatrix::Identity();

    Coordinates desired_pos;
    desired_pos = initial_pos;
    RotationMatrix desired_rot;
    desired_rot = initial_rot;
    JointStateVector desired_joints;

    for (int i = 0; i < 100; i++)
    {
        desired_pos += (initial_pos - final_pos) / 100;
        desired_rot += (initial_rot - final_rot) / 100;

        controller.ur5_move_to(desired_pos, desired_rot, false);

    }







    return 0;

    Coordinates dest;
    dest << 0.3, 0.3, -0.3;
    RotationMatrix rot = RotationMatrix::Identity();

    double linear_space[100];
    for (int i = 0; i < 100; i++)
    {
        linear_space[i] = (double)i * 5. / 100.;
    }

    auto xe = [](double t) {
        Coordinates c;
        c << 0.4 * sin(2 * M_PI * t), 0.4 * cos(2 * M_PI * t), 0.5;
        // c << 0.4 * t, 0.4 * (1 - t), 0.5;
        return c;
    };


    for (int i = 0; i < 100; i++)
    {
        Coordinates x = xe(linear_space[i]);
        controller.ur5_move_to(x, rot, false);
        // if (i % 2 == 0) {
        //     controller.ur5_set_gripper(50);
        // } else {
        //     controller.ur5_set_gripper(100);
        // }
    }
*/
    return 0;
}