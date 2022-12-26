#include "ur5_controller/ur5_controller_lib.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>

/* Public functions */

UR5Controller::UR5Controller(double loop_frequency, double joints_error, double settling_time) : loop_rate(loop_frequency)
{
    this->loop_frequency = loop_frequency;
    this->joints_error = joints_error;
    this->settling_time = settling_time;

    // Set params from ros param server
    node.getParam("/real_robot", is_real_robot);
    node.getParam("/soft_gripper", using_soft_gripper);
    node.getParam("/gripper_sim", is_simulating_gripper);

    // Publisher initialization
    joint_state_pub = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1000);
    gripper_state_pub = node.advertise<std_msgs::Int32>("/ur5/gripper_controller/command", 1);

    // Subscriber initialization
    joint_state_sub = node.subscribe("/ur5/joint_states", 1, &UR5Controller::joint_state_callback, this);
}

void UR5Controller::set_gripper(int diameter)
{
    gripper_diameter = diameter;
    send_gripper_state(diameter);
}

void UR5Controller::get_joint_states(JointStateVector &joints)
{
    ros::spinOnce();
    joints = current_joints;
}

/* Private functions */

void UR5Controller::joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    int n; // number of joints    
    if (is_real_robot || !is_simulating_gripper)
        n = 6;
    else
        n = using_soft_gripper ? 8 : 9;

    // Get joints values from topic
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            if (joint_names[j].compare(msg->name[i]) == 0)
                current_joints(j) = msg->position[i];
        }
        
        // Gripper
        for (int j = 6; j < n; j++)
        {
            if (joint_names[j].compare(msg->name[i]) == 0)
                current_gripper(j - 6) = msg->position[i];
        }
    }
}

void UR5Controller::send_joint_state(JointStateVector &desired_joints)
{
    std_msgs::Float64MultiArray joint_state_msg_array;
    if (is_real_robot || !is_simulating_gripper)
    {
        // Cannot send gripper joints values to real robot
        joint_state_msg_array.data.resize(6);
    }
    else 
    {
        int n = using_soft_gripper ? 2 : 3; // number of fingers

        joint_state_msg_array.data.resize(6 + n);
        // Add the state of the gripper 
        for (int i = 0; i < n; i++)
            joint_state_msg_array.data[6 + i] = current_gripper(i);
    }

    // Add state of the joints
    for (int i = 0; i < 6; i++)
        joint_state_msg_array.data[i] = desired_joints[i];

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