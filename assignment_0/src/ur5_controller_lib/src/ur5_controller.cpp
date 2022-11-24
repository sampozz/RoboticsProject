#include "ur5_controller_lib/ur5_controller.h"
#include <std_msgs/Float64MultiArray.h>

static std::string joint_names[] = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

UR5Controller::UR5Controller(int argc, char** argv)
{
    // ROS Node initialization 
    ros::init(argc, argv, "ur5_controller");
    
    // Publisher initialization
    joint_state_pub = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1000);

    // Subscriber initialization
    // joint_state_sub = node.subscribe("/ur5/joint_states", 1000, joint_state_callback);
}

void UR5Controller::joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    // Get joints values from topic
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            if (joint_names[j].compare(msg->name[i]) == 0) {
                current_pos[j] = msg->position[i];
            }
        }
    }
}

void UR5Controller::ur5_move_to(Coordinates &pos)
{
}