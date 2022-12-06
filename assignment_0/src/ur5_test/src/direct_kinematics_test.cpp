#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "kinematics_lib/ur5_kinematics.h"
#include <iostream>

std::string joint_names[] = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

void compute_direct_kin(const sensor_msgs::JointState::ConstPtr& msg)
{
    JointStateVector th;
    Coordinates pos;
    RotationMatrix rot;

    // # Get joints values from topic
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            if (joint_names[j].compare(msg->name[i]) == 0) {
                th[j] = msg->position[i];
            }
        }
    }

    // Compute kinematics
    ur5_direct(th, pos, rot);

    std::cout << "Position: " << pos.transpose() << std::endl;
    std::cout << "Rotation: " << rot << std::endl;
}

int main(int argc, char** argv)
{
    // Init node
    ros::init(argc, argv, "direct_kin_test");
    ros::NodeHandle node;
    
    // Subscribe to topic: listen to ur5 joints values
    ros::Subscriber sub = node.subscribe("/ur5/joint_states", 1000, compute_direct_kin);
    
    // Wait for callbacks
    ros::spin();
    
    return 0;
}