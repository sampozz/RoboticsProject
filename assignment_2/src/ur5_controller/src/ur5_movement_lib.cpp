#include "ur5_controller/ur5_controller_lib.h"
#include <map>
#include <Eigen/SVD>

using namespace std;

static JointStateVector lin_filter;
static double v_ref;

int *sort_ik_result(Eigen::Matrix<double, 8, 6> &ik_result, JointStateVector &initial_joints);

/* Public functions */

bool UR5Controller::ur5_move_to(Coordinates &pos, RotationMatrix &rot, int n)
{
    // Read the /ur5/joint_states topic and get the initial configuration
    ros::spinOnce();
    JointStateVector initial_joints = current_joints;
    ROS_INFO("Moving UR5: initial joints values: %.2f %.2f %.2f %.2f %.2f %.2f", initial_joints(0), initial_joints(1), initial_joints(2),
        initial_joints(3), initial_joints(4), initial_joints(5)); 

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
        ROS_WARN("UR5 could not find a valid path!");
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
    ROS_INFO("Moving UR5: final joints values: %.2f %.2f %.2f %.2f %.2f %.2f", current_joints(0), current_joints(1), current_joints(2),
        current_joints(3), current_joints(4), current_joints(5)); 
    return true;
}

/* Private functions */

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

        if (intermediate_testing_joints.norm() == 0)
            continue;

        // Check position constraints
        if (testing_position(2) > 0.74)
            return false;

        Eigen::Matrix<double, 6, 6> jac;
        ur5_jacobian(intermediate_testing_joints, jac);
        
        // Check singularity with jacobian determinant
        if (abs(jac.determinant()) < 0.00001)
            return false;

        // Check singulaity with jacobian singular values
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(jac, Eigen::ComputeThinU | Eigen::ComputeThinV);
        if (abs(svd.singularValues()(5)) < 0.00001)
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

void UR5Controller::init_filters()
{
    ros::spinOnce();
    lin_filter = current_joints;
    v_ref = 0.0;
}

JointStateVector UR5Controller::linear_filter(const JointStateVector &final_pos)
{
    double v_des = 0.6;
    v_ref += 0.005 * (v_des - v_ref);
    lin_filter += 1.0 / loop_frequency * v_ref * (final_pos - lin_filter) / (final_pos - lin_filter).norm();
    return lin_filter;
}

double norm_angle(double angle)
{
    if (angle > 0)
        angle = fmod(angle, 2 * M_PI);
    else
        angle = 2 * M_PI - fmod(-angle, 2 * M_PI);
    return angle;
}

double UR5Controller::compute_error(JointStateVector &desired_joints, JointStateVector &current_joints)
{
    JointStateVector current_normed, desired_normed;
    for (int i = 0; i < 6; i++)
    {
        current_normed(i) = norm_angle(current_joints(i));
        desired_normed(i) = norm_angle(desired_joints(i));
    }
    return (current_normed - desired_normed).norm();
}