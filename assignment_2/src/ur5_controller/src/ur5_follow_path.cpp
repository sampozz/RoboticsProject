#include "ur5_controller/ur5_controller_lib.h"
#include <map>

using namespace std;

int *sort_ik_result(Eigen::Matrix<double, 8, 6> &ik_result, JointStateVector &initial_joints);

bool UR5Controller::ur5_follow_path(Coordinates &pos, RotationMatrix &rot, int n)
{
    // Read the /ur5/joint_states topic and get the initial configuration
    ros::spinOnce();
    JointStateVector initial_joints = current_joints;
    std::cout << "Initial joints values: " << initial_joints.transpose() << std::endl;

    // Compute complete inverse kinematics to find all the possibile final configurations
    Eigen::Matrix<double, 8, 6> ik_result;
    ur5_inverse_complete(pos, rot, ik_result);
    int *indexes = sort_ik_result(ik_result, initial_joints);

    double *path;
    bool is_valid = false;

    // Compute the path of for every ik solution
    for (int i = 0; i < 8; i++)
    {
        int index = indexes[i];
        JointStateVector final_testing_joints;
        final_testing_joints << ik_result(index, 0), ik_result(index, 1), ik_result(index, 2),
            ik_result(index, 3), ik_result(index, 4), ik_result(index, 5);

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
        return false;
    }

    // Filter configuration
    init_filters();

    // Movement loop
    int i = -1;
    while (++i < n)
    {
        JointStateVector intermediate_joints, desired_joints;
        intermediate_joints << path[i * 6], path[i * 6 + 1], path[i * 6 + 2],
            path[i * 6 + 3], path[i * 6 + 4], path[i * 6 + 5];

        if (intermediate_joints.norm() == 0)
            continue;

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
    return true;
}

bool UR5Controller::validate_path(double *path, int n)
{
    // Compute direct kinematics on every configuration inside the path
    for (int j = 0; j < n; j++)
    {
        Coordinates testing_position;
        RotationMatrix testing_rotation;
        JointStateVector intermediate_testing_joints;

        intermediate_testing_joints << path[j * 6], path[j * 6 + 1], path[j * 6 + 2],
            path[j * 6 + 3], path[j * 6 + 4], path[j * 6 + 5];
        ur5_direct(intermediate_testing_joints, testing_position, testing_rotation);

        // Check position constraints
        if (testing_position(2) > 0.74)
            return false;
    }
    return true;
}

int *sort_ik_result(Eigen::Matrix<double, 8, 6> &ik_result, JointStateVector &initial_joints)
{
    map<double, int> m;
    for (int i = 0; i < 8; i++)
    {
        JointStateVector comp;
        comp << ik_result(i, 0), ik_result(i, 1), ik_result(i, 2), 
            ik_result(i, 3), ik_result(i, 4), ik_result(i, 5);
        m.insert(pair<double, int>((comp - initial_joints).norm(), i));
    }
    
    int *list = (int *)malloc(sizeof(int) * 8);
    int i = 0;
    for (auto const& it : m) {
        list[i++] = it.second;
    }
    return list;
}