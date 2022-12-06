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
    ros::Publisher gripper_state_pub;
    ros::Subscriber joint_state_sub;

    double settling_time;
    double loop_frequency;
    double joints_error;

    std::string joint_names[9] = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
                                  "hand_1_joint", "hand_2_joint", "hand_3_joint"};

    JointStateVector current_joints;
    GripperStateVector current_gripper;

    int gripper_diameter;

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
     * Send selected diameter to /ur5/gripper_controller/command
     */
    void send_gripper_state(int diameter);

    /**
     * Compute joint values by following consecutive approximations.
     * The velocity of the end effector reamins constant.
     *  Use this function iteratively to send joint states to the robot and avoid steps
     */
    JointStateVector linear_filter(const JointStateVector &final_pos);

    /**
     * Compute joint values by following consecutive approximations.
     * The velocity of the end effector gradually diminishes when reaching the final position.
     * Use this function iteratively to send joint states to the robot and avoid steps
     */
    JointStateVector second_order_filter(const JointStateVector &final_pos);

    /**
     * Init values of linear and so filters 
     */
    void init_filters();

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

    /**
     * For every joint configuration, compute direct kinematics and check if the posititon collides with the workbanch
     * @return true if path is valid
     */
    bool validate_path(double *path, int n);

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
     * @param pos Final cartesian position of the end effector
     * @param rot Final rotation of the end effector
     * @param select_filter false => linear filter, true => second order filter
     * This function follows the procedure:
     * 1. read the /ur5/joint_states topic and get the initial configuration
     * 2. compute inverse kinematics to get desired joint values
     * 3. compute a filter for a smooth transition
     * 4. publish joints values to topic at each iteration
     */
    void ur5_move_to(Coordinates &pos, RotationMatrix &rot, bool select_filter);

    /**
     * Move end effector to desired position (pos) and rotation (rot) by following a path computed by the kinematics libray
     * @param pos Final cartesian position of the end effector
     * @param rot Final rotation of the end effector
     * @param n Number of intermediate points
     * @return true if path was valid and movement succeded
     * This function follows the procedure:
     * 1. Read the /ur5/joint_states topic and get the initial configuration 
     * 2. Compute complete inverse kinematics to find all the possibile final configurations
     * 3. Compute the path of for every ik solution
     * 4. Compute direct kinematics on every configuration inside the path and check position constraints
     * 5. If the path is acceptable, follow it 
     */
    bool ur5_follow_path(Coordinates &pos, RotationMatrix &rot, int n);

    /**
     *  Open and close the gripper at the selected diameter
     */
    void ur5_set_gripper(int diameter);

    /**
     * Save current joint state vector in the parameter joints.
     */
    void ur5_get_joint_states(JointStateVector &joints);
};

#endif