#ifndef __UR5_CONTROLLER_H__
#define __UR5_CONTROLLER_H__

#include "ur5_kinematics_lib/ur5_kinematics.h"
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>

class UR5Controller
{
private:
    ros::NodeHandle node;
    ros::Rate loop_rate;
    ros::Publisher joint_state_pub;
    ros::Subscriber joint_state_sub;

    double settling_time;
    double loop_frequency;
    double joints_error;

    std::string joint_names[9] = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
                                  "hand_1_joint", "hand_2_joint", "hand_3_joint"};

    JointStateVector filter[2];
    JointStateVector current_joints;
    GripperStateVector current_gripper;

    bool is_grabbing = false;

    /**
     * Callback function, listen to /ur5/joint_states topic and update current_pos
     */
    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg);

    /**
     * Convert the JointStateVector desired_pos to a Float64MultiArray,
     * then send the array to ros topic defined by the joint_state_pub publisher
     */
    void send_joint_state(JointStateVector &desired_pos);

    /**
     * Compute joint values by following consecutive approximations.
     * Use this function iteratively to send joint states to the robot and avoid steps
     */
    JointStateVector second_order_filter(const JointStateVector &final_pos);

    /**
     * Check if the desired state vector is equal to the current vector by comparing
     * the normalized values of the angles in the range [0, 2 * pi].
     * If check is succesful, replace values in the current_joints vector
     */
    void adjust_desired_joints(JointStateVector &current_joints, JointStateVector &desired_joints);

    /**
     * Compute difference between the two vector, by normalizing the angles.
     * Eg. -PI/2 will be equal to 3*PI/2
     */
    double compute_error(JointStateVector &current_joints, JointStateVector &desired_joints);

public:
    /**
     * Constructor. Initialize ros publishers and subscribers.
     * @param loop_frequency Specifies the rate of received and sent instruction in a movement loop.
     * @param joints_error Acceptable error between desired position and effective position at the end of a movement operation. Higher value => less precision.
     * @param settling_time Required time to complete a movement operation. Higher value => higher speed.
     */
    UR5Controller(double loop_frequency, double joints_error, double settling_time);

    /**
     * Move end effector to desired position (pos) and rotation (rot)
     * This function follows the procedure:
     * 1. read the /ur5/joint_states topic and get the initial position
     * 2. compute inverse kinematics to get desired joint values
     * 3. compute a filter for a smooth transition
     * 4. publish joints values to topic at each iteration
     * 5. end
     */
    void ur5_move_to(Coordinates &pos, RotationMatrix &rot);
};

#endif