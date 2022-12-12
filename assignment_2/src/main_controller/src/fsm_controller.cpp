#include "ros/ros.h"
#include "main_controller/fsm.h"
#include "ur5_controller/MoveTo.h"
#include "ur5_controller/SetGripper.h"
#include "shelfino_controller/MoveTo.h"
#include "gazebo_ros_link_attacher/Attach.h"

using namespace std;

StateMachine_t fsm[] = {
    {STATE_INIT, init},
    {STATE_UR5_HOME, ur5_homing},
    {STATE_UR5_LOAD, ur5_load},
    {STATE_UR5_UNLOAD, ur5_unload},
    {STATE_SHELFINO_TEST, shelfino_test},
};

int main(int argc, char **argv)
{
    // ROS Node initialization
    ros::init(argc, argv, "fsm_controller");
    ros::NodeHandle fsm_node;

    ROS_ERROR("Starting node...");

    ur5_move_client = fsm_node.serviceClient<ur5_controller::MoveTo>("ur5/move_to");
    ur5_gripper_client = fsm_node.serviceClient<ur5_controller::SetGripper>("ur5/set_gripper");
    shelfino_move_client = fsm_node.serviceClient<shelfino_controller::MoveTo>("shelfino/move_to");
    gazebo_link_attacher = fsm_node.serviceClient<gazebo_ros_link_attacher::Attach>("link_attacher_node/attach");
    gazebo_link_detacher = fsm_node.serviceClient<gazebo_ros_link_attacher::Attach>("link_attacher_node/detach");

    // Initial state
    current_state = STATE_INIT;

    // Wait a bit, just to avoid shit
    ros::Duration(0.5).sleep();

    while (ros::ok())
    {
        if (current_state < STATE_END)
        {
            cout << "Executing state function " << current_state << endl;
            (*fsm[current_state].state_function)();
        }
        else
        {
            break;
        }
    }

    return 0;
}
