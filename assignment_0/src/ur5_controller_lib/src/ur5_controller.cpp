#include "ur5_controller_lib/ur5_controller.h"
#include <std_msgs/Float64MultiArray.h>

/* Public functions */

UR5Controller::UR5Controller() : loop_rate(1000.)
{
    // Publisher initialization
    joint_state_pub = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1000);

    // Subscriber initialization
    joint_state_sub = node.subscribe("/ur5/joint_states", 1, &UR5Controller::joint_state_callback, this);
}

void UR5Controller::ur5_move_to(Coordinates &pos, RotationMatrix &rot)
{
    // Read the /ur5/joint_states topic and get the initial position
    ros::spinOnce();
    JointStateVector desired_joints = current_joints;
    std::cout << "Initial joints values: " << current_joints.transpose() << std::endl;

    // Compute inverse kinematics to get desired joint values
    JointStateVector final_joints;
    ur5_inverse(pos, rot, final_joints);
    std::cout << "Desired joints values: " << final_joints.transpose() << std::endl;

    // Init a filter for smooth transition
    filter[0] = filter[1] = desired_joints;

    // Movement loop
    double error = (current_joints - final_joints).norm();
    while (ros::ok() && error > 0.005)
    {
        // Compute filter
        desired_joints = secondOrderFilter(final_joints);

        // Send position to topic
        send_joint_state(desired_joints);

        // Loop state
        loop_rate.sleep();
        ros::spinOnce();

        // Compute error between current position and final position
        error = (current_joints - final_joints).norm();
    }

    std::cout << "Final joints values: " << current_joints.transpose() << std::endl;
    std::cout << "Error: " << error << std::endl;
}

/* Private functions */

void UR5Controller::joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    // Get joints values from topic
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            if (joint_names[j].compare(msg->name[i]) == 0)
            {
                current_joints(j) = msg->position[i];
            }
        }
    }
}

void UR5Controller::send_joint_state(JointStateVector &desired_pos)
{
    std_msgs::Float64MultiArray joint_state_msg_array;
    joint_state_msg_array.data.resize(6);

    // Create message object
    for (int i = 0; i < 6; i++)
    {
        joint_state_msg_array.data[i] = desired_pos[i];
    }
    // Publish desired joint states message
    joint_state_pub.publish(joint_state_msg_array);
}

JointStateVector UR5Controller::secondOrderFilter(const JointStateVector &final_pos)
{
    double dt = 1 / loop_frequency;
    double gain = dt / (0.1 * settling_time + dt);
    filter[0] = (1 - gain) * filter[0] + gain * final_pos;
    filter[1] = (1 - gain) * filter[1] + gain * filter[0];
    return filter[1];
}