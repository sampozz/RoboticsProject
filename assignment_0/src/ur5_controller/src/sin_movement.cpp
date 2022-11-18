#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>


typedef Eigen::Matrix<double, 6, 1> JointStateVector;


int main(int argc, char** argv)
{
    // Initialize ROS Node
    ros::init(argc, argv, "sin_movement");
    ros::NodeHandle node;

    // Declare to publish to a topic and get a Publisher object (publisher destination joint state)
    ros::Publisher pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1000);

    // Variables
    double loop_frequency = 1000.;
    double loop_time = 0.;

    std_msgs::Float64MultiArray jointState_msg_array; 
    jointState_msg_array.data.resize(6);
    
    JointStateVector q_des = JointStateVector::Zero();
    JointStateVector q_des0 = JointStateVector::Zero();

    JointStateVector amp;
    JointStateVector freq;
    amp << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    freq << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2;

    // How long will be the rate??
    ros::Rate loop_rate(loop_frequency);

    // Init value (find it in params.py)
    q_des0 << -0.3223527113543909, -0.7805794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085, -1.0017417112933558;
    
    while (ros::ok())
    {
        // Generate sin wave
        q_des = q_des0.array() + amp.array() * (2 * M_PI * freq * loop_time).array().sin();

        // Create message object
        for (int i = 0; i < q_des.size(); i++)
        {
            jointState_msg_array.data[i] = q_des[i];
        }
        // Publish desired joint states message
        pub_des_jstate.publish(jointState_msg_array);
        
        loop_time += (double)1 / loop_frequency;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}