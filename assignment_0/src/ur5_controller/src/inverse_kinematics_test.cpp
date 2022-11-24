#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "kinematics_lib/ur5_kinematics.h"
#include "std_msgs/Float64MultiArray.h"
#include <iostream>

// Global variables
JointStateVector filter_1 = JointStateVector::Zero();
JointStateVector filter_2 = JointStateVector::Zero();

JointStateVector secondOrderFilter(const JointStateVector& input, const double rate, const double settling_time)
{
    double dt = 1 / rate;
    double gain =  dt / (0.1 * settling_time + dt);
    filter_1 = (1 - gain) * filter_1 + gain * input;
    filter_2 = (1 - gain) * filter_2 + gain * filter_1;
    return filter_2;
}

int main(int argc, char** argv)
{
    // Initialize ROS Node
    ros::init(argc, argv, "inverse_kin");
    ros::NodeHandle node;

    // Declare to publish to a topic and get a Publisher object (publisher destination joint state)
    ros::Publisher pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1000);

    // msg to write to the ros topic
    std_msgs::Float64MultiArray jointState_msg_array; 
    jointState_msg_array.data.resize(6);

    // How long will be the rate??
    double loop_frequency = 1000.;
    ros::Rate loop_rate(loop_frequency);

    // Initial joints position (find it in params.py)
    JointStateVector q_des0 = JointStateVector::Zero();
    q_des0 << -0.3223527113543909, -0.7805794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085, -1.0017417112933558;

    // Final joints position, this is the only parameter of the direct kinematics (this is a random value)
    JointStateVector q_des1 = JointStateVector::Zero();
    Coordinates dest;
    dest << 0.1978, -0.6579, 0.2324;
    RotationMatrix rot;
    rot << 0.2990, -0.8513, 0.4321, -0.6107, 0.1766, 0.7720, -0.7333, -0.4941, -0.4670;
    ur5_inverse(dest, rot, q_des1);
    std::cout << "DEST: " << q_des1 << std::endl << std::endl;

    // Desired position, this will be converted to Float64MultiArray and sent to ros topic
    JointStateVector q_des = JointStateVector::Zero();

    // Initialize filter values
    filter_1 = q_des0;
    filter_2 = q_des0;
    
    while (ros::ok())
    {
        // Calculate next position to send to the robot
        q_des = secondOrderFilter(q_des1, loop_frequency, 5.);

        // Create message object
        for (int i = 0; i < q_des.size(); i++)
        {
            jointState_msg_array.data[i] = q_des[i];
        }
        // Publish desired joint states message
        pub_des_jstate.publish(jointState_msg_array);

        // Loop state
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}