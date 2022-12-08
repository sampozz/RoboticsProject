#include "ur5_controller/ur5_controller_lib.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>

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
    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            if (joint_names[j].compare(msg->name[i]) == 0)
                current_joints(j) = msg->position[i];
        }
        // Gripper
        for (int j = 6; j < 8; j++)
        {
            if (joint_names[j].compare(msg->name[i]) == 0)
                current_gripper(j - 6) = msg->position[i];
        }
    }
}

void UR5Controller::send_joint_state(JointStateVector &desired_pos)
{
    std_msgs::Float64MultiArray joint_state_msg_array;
    joint_state_msg_array.data.resize(8);

    // Create message object
    for (int i = 0; i < 6; i++)
        joint_state_msg_array.data[i] = desired_pos[i];

    // Add the state of the gripper (remember to resize data array)
    for (int i = 0; i < 2; i++)
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