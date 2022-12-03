#include "ur5_controller_lib/ur5_controller.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>

static JointStateVector so_filter[2];
static JointStateVector lin_filter;
static double v_ref;

/* Public functions */

UR5Controller::UR5Controller(double loop_frequency, double joints_error, double settling_time) : loop_rate(loop_frequency)
{
    this->loop_frequency = loop_frequency;
    this->joints_error = joints_error;
    this->settling_time = settling_time;

    // Publisher initialization
    joint_state_pub = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1000);
    gripper_state_pub = node.advertise<std_msgs::Int32>("/ur5/gripper_controller/command", 1);

    // Subscriber initialization
    joint_state_sub = node.subscribe("/ur5/joint_states", 1, &UR5Controller::joint_state_callback, this);
}

void UR5Controller::ur5_move_to(Coordinates &pos, RotationMatrix &rot, bool select_filter)
{
    // Read the /ur5/joint_states topic and get the initial configuration
    ros::spinOnce();
    JointStateVector desired_joints = current_joints;
    std::cout << "Initial joints values: " << current_joints.transpose() << std::endl;

    // Compute inverse kinematics to get desired joint values
    JointStateVector final_joints;
    ur5_inverse(pos, rot, final_joints);
    std::cout << "Desired joints values: " << final_joints.transpose() << std::endl;

    // Check if initial joints angles are equal to the computed angles
    adjust_desired_joints(final_joints, desired_joints);
    std::cout << "Adjusted joints values: " << final_joints.transpose() << std::endl;

    // Init supported filters parameters
    so_filter[0] = so_filter[1] = lin_filter = desired_joints;
    v_ref = 0.0;

    // Movement loop
    while (ros::ok() && compute_error(final_joints, current_joints) > joints_error)
    {
        // Compute filter
        if (select_filter)
            desired_joints = second_order_filter(final_joints);
        else
            desired_joints = linear_filter(final_joints);

        // Send position to topic
        send_joint_state(desired_joints);

        // Loop state
        loop_rate.sleep();
        ros::spinOnce();
    }
    std::cout << "Final joints values: " << current_joints.transpose() << std::endl;
}

void UR5Controller::ur5_set_gripper(int diameter)
{
    gripper_diameter = diameter;
    send_gripper_state(diameter);
}

void UR5Controller::ur5_get_joint_states(JointStateVector &joints)
{
    ros::spinOnce();
    joints = current_joints;
}

/* Private functions */

void UR5Controller::joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    // Get joints values from topic
    for (int i = 0; i < 9; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            if (joint_names[j].compare(msg->name[i]) == 0)
                current_joints(j) = msg->position[i];
        }
        for (int j = 6; j < 9; j++)
        {
            if (joint_names[j].compare(msg->name[i]) == 0)
                current_gripper(j - 6) = msg->position[i];
        }
    }
}

void UR5Controller::send_joint_state(JointStateVector &desired_pos)
{
    std_msgs::Float64MultiArray joint_state_msg_array;
    joint_state_msg_array.data.resize(9);

    // Create message object
    for (int i = 0; i < 6; i++)
        joint_state_msg_array.data[i] = desired_pos[i];

    // Add the state of the gripper
    for (int i = 0; i < 3; i++)
        joint_state_msg_array.data[i + 6] = current_gripper(i);

    // Publish desired joint states message
    joint_state_pub.publish(joint_state_msg_array);
}

void UR5Controller::send_gripper_state(int diameter)
{
    std_msgs::Int32 diameter_msg;
    diameter_msg.data = diameter;

    // Publish desired gripper state message
    gripper_state_pub.publish(diameter_msg);

    // Locosim manages this movement, wait a bit
    ros::Duration(2.0).sleep();
}

void UR5Controller::init_filters()
{
    ros::spinOnce();
    so_filter[0] = so_filter[1] = lin_filter = current_joints;
    v_ref = 0.0;
}

JointStateVector UR5Controller::linear_filter(const JointStateVector &final_pos)
{
    double v_des = 0.6;
    v_ref += 0.005 * (v_des - v_ref);
    lin_filter += 1.0 / loop_frequency * v_ref * (final_pos - lin_filter) / (final_pos - lin_filter).norm();
    return lin_filter;
}

JointStateVector UR5Controller::second_order_filter(const JointStateVector &final_pos)
{
    double dt = 1 / loop_frequency;
    double gain = dt / (0.1 * settling_time + dt);
    so_filter[0] = (1 - gain) * so_filter[0] + gain * final_pos;
    so_filter[1] = (1 - gain) * so_filter[1] + gain * so_filter[0];
    return so_filter[1];
}

double norm_angle(double angle)
{
    if (angle > 0)
        angle = fmod(angle, 2 * M_PI);
    else
        angle = 2 * M_PI - fmod(-angle, 2 * M_PI);
    return angle;
}

void UR5Controller::adjust_desired_joints(JointStateVector &desired_joints, JointStateVector &current_joints)
{
    double current_norm, desired_norm;
    for (int i = 0; i < 6; i++)
    {
        current_norm = norm_angle(current_joints(i));
        desired_norm = norm_angle(desired_joints(i));
        // std::cout << "current: " << current_norm << " desired: " << desired_norm << std::endl;
        if (current_norm >= desired_norm - 0.01 && current_norm <= desired_norm + 0.01)
            current_joints(i) = desired_joints(i);
    }
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