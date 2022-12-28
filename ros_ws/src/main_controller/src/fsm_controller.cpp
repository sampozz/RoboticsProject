#include "main_controller/fsm.h"
#include <ros/console.h>

using namespace std;

/* Extern variables */

extern ros::ServiceClient shelfino_move_client, shelfino_point_client,
    shelfino_rotate_client, shelfino_forward_client, 
    gazebo_link_attacher, gazebo_link_detacher,
    ur5_move_client, ur5_gripper_client,
    detection_client, gazebo_model_state;

extern std::vector<std::vector<double>> areas;

/* FSM Functions arrays for the three assignemnts */

StateMachine_t fsm_ass_1[] = {};

StateMachine_t fsm_ass_2[] = {
    {STATE_INIT, ass_2::init},
    {STATE_SHELFINO_ROTATE_AREA, ass_2::shelfino_rotate_towards_next_area},
    {STATE_SHELFINO_NEXT_AREA, ass_2::shelfino_next_area},
    {STATE_SHELFINO_SEARCH_BLOCK, ass_2::shelfino_search_block},
    {STATE_SHELFINO_CHECK_BLOCK, ass_2::shelfino_check_block},
    {STATE_UR5_LOAD, ass_2::ur5_load},
    {STATE_UR5_UNLOAD, ass_2::ur5_unload},
};

StateMachine_t fsm_ass_3[] = {};

/* Global variables */

ros::ServiceClient detection_client, gazebo_model_state;
State_t current_state;
std::vector<std::vector<double>> areas;

void get_world_params(ros::NodeHandle& n)
{
    for (int i = 0; i < 4; i++) 
        areas.push_back({});

    n.getParam("area0", areas[0]);
    n.getParam("area1", areas[1]);
    n.getParam("area2", areas[2]);
    n.getParam("area3", areas[3]);
}

int main(int argc, char **argv)
{
    // Arguments validation
    if (argc < 2)
    {
        ROS_ERROR("Assignment number not provided. Exiting...");
        return 1;
    }
    
    int assignment_number = std::stoi(argv[1]);

    if (assignment_number < 1 || assignment_number > 3)
    {
        ROS_ERROR("Assignment number is wrong. Exiting...");
        return 1;
    }

    ROS_INFO("Executing assignment %d", assignment_number);

    // ROS Node initialization
    ros::init(argc, argv, "fsm_controller");
    ros::NodeHandle fsm_node;
    ros::Rate loop_rate(100.);

    // Setup services
    ur5_move_client = fsm_node.serviceClient<ur5_controller::MoveTo>("ur5/move_to");
    ur5_gripper_client = fsm_node.serviceClient<ur5_controller::SetGripper>("ur5/set_gripper");
    shelfino_move_client = fsm_node.serviceClient<shelfino_controller::MoveTo>("shelfino/move_to");
    shelfino_rotate_client = fsm_node.serviceClient<shelfino_controller::Rotate>("shelfino/rotate");
    shelfino_point_client = fsm_node.serviceClient<shelfino_controller::PointTo>("shelfino/point_to");
    shelfino_forward_client = fsm_node.serviceClient<shelfino_controller::MoveForward>("shelfino/move_forward");

    // Vision services
    detection_client = fsm_node.serviceClient<yolov5_ros::Detect>("vision/detect");

    // Gazebo services
    gazebo_model_state = fsm_node.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_link_attacher = fsm_node.serviceClient<gazebo_ros_link_attacher::Attach>("link_attacher_node/attach");
    gazebo_link_detacher = fsm_node.serviceClient<gazebo_ros_link_attacher::Attach>("link_attacher_node/detach");

    // Get world params
    get_world_params(fsm_node);

    // Initial state
    current_state = STATE_INIT;

    // Wait a bit, just to avoid shit
    ros::Duration(0.5).sleep();

    while (ros::ok())
    {
        if (current_state < STATE_END)
        {
            ROS_DEBUG("Executing state function %d", current_state);

            if (assignment_number == 1)
                (*fsm_ass_1[current_state].state_function)();      
            else if (assignment_number == 2)
                (*fsm_ass_2[current_state].state_function)();      
            else
                (*fsm_ass_3[current_state].state_function)();      

            ros::spinOnce();
            loop_rate.sleep();
        }
        else
        {
            ROS_INFO("Mission completed!");
            break;
        }
    }

    return 0;
}
