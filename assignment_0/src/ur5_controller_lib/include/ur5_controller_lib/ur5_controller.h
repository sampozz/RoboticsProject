#ifndef __UR5_CONTROLLER_H__
#define __UR5_CONTROLLER_H__

#include "ur5_kinematics_lib/ur5_kinematics.h"
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>

class UR5Controller
{
private:
    ros::NodeHandle node;
    ros::Publisher joint_state_pub;
    ros::Subscriber joint_state_sub; 
    Coordinates current_pos;

    /**
     * Callback function, listen to /ur5/joint_states topic and update current_pos
     */
    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg);

public:
    UR5Controller(int argc, char** argv);

    /**
     * Move end effector to desired position (pos), following the procedure:
     * 1. read the /ur5/joint_states topic and get the initial position
     * 2. compute inverse kinematics to get desired joint values
     * 3. compute a filter for a smooth transition
     * 4. publish joints values to topic at each iteration
     * 5. end
     */
    void ur5_move_to(Coordinates &pos);
};

#endif