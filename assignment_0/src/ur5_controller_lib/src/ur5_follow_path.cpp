#include "ur5_controller_lib/ur5_controller.h"

void UR5Controller::ur5_follow_path(Coordinates &pos, RotationMatrix &rot, int n)
{
    // Read the /ur5/joint_states topic and get the initial configuration
    ros::spinOnce();
    JointStateVector initial_joints = current_joints;
    std::cout << "Initial joints values: " << initial_joints.transpose() << std::endl;

    // Compute complete inverse kinematics to find all the possibile final configurations
    Eigen::Matrix<double, 8, 6> ik_result;
    ur5_inverse_complete(pos, rot, ik_result);

    double *path;
    bool is_valid = false;

    // Compute the path of for every ik solution
    for (int i = 0; i < 8; i++)
    {
        JointStateVector final_testing_joints;
        final_testing_joints << ik_result(i, 0), ik_result(i, 1), ik_result(i, 2),
            ik_result(i, 3), ik_result(i, 4), ik_result(i, 5);

        path = ur5_motion_plan(initial_joints, final_testing_joints, n);

        if (validate_path(path, n))
        {
            is_valid = true;
            break;
        }
    }

    if (!is_valid)
    {
        std::cout << "Could not find a valid path" << std::endl;
        return;
    }

    // Now *path contains a valid path to follow
    std::cout << "Accettable path found!" << std::endl;

    // Movement loop
    int i = -1;
    while (++i < n)
    {
        JointStateVector intermediate_joints, desired_joints;
        intermediate_joints << path[i * 6], path[i * 6 + 1], path[i * 6 + 2],
            path[i * 6 + 3], path[i * 6 + 4], path[i * 6 + 5];

        if (intermediate_joints.norm() == 0)
            continue;

        // Filter configuration
        init_filters();

        // Movement loop (between two intermediate points)
        while (ros::ok() && compute_error(current_joints, intermediate_joints) > joints_error)
        {
            desired_joints = linear_filter(intermediate_joints);

            // Send position to topic
            send_joint_state(desired_joints);

            // Loop state
            loop_rate.sleep();
            ros::spinOnce();
        }
    }
    std::cout << "Final joints values: " << current_joints.transpose() << std::endl;
}
