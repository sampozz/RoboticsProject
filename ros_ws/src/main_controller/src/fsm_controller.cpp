#include "main_controller/fsm.h"
#include <ros/console.h>

using namespace std;

/* Extern variables */

extern ros::ServiceClient shelfino_move_client, shelfino_point_client,
    shelfino_rotate_client, shelfino_forward_client, 
    gazebo_link_attacher, gazebo_link_detacher,
    ur5_move_client, ur5_gripper_client,
    detection_client, gazebo_set_state, 
    gazebo_get_state, vision_stop_client, 
    pointcloud_client;

extern std::vector<std::vector<double>> areas;
extern bool real_robot;

/* FSM Functions arrays for the three assignments */

StateMachine_t fsm_test = {
    {STATE_INIT, state_test},
};

StateMachine_t fsm_ass_1 = {
    {STATE_INIT, ass_1::init},
    {STATE_SHELFINO_ROTATE_AREA, ass_1::shelfino_rotate_towards_next_area},
    {STATE_SHELFINO_NEXT_AREA, ass_1::shelfino_next_area},
    {STATE_SHELFINO_SEARCH_BLOCK, ass_1::shelfino_search_block},
    {STATE_SHELFINO_CHECK_BLOCK, ass_1::shelfino_check_block},
};

StateMachine_t fsm_ass_2 = {
    {STATE_INIT, ass_2::init},
    {STATE_SHELFINO_ROTATE_AREA, ass_2::shelfino_rotate_towards_next_area},
    {STATE_SHELFINO_NEXT_AREA, ass_2::shelfino_next_area},
    {STATE_SHELFINO_SEARCH_BLOCK, ass_2::shelfino_search_block},
    {STATE_SHELFINO_CHECK_BLOCK, ass_2::shelfino_check_block},
    {STATE_UR5_LOAD, ass_2::ur5_load},
    {STATE_UR5_UNLOAD, ass_2::ur5_unload},
};

StateMachine_t fsm_ass_3 = {
    {STATE_INIT, ass_3::init},
    {STATE_SHELFINO_ROTATE_AREA, ass_3::shelfino_rotate_towards_next_area},
    {STATE_SHELFINO_NEXT_AREA, ass_3::shelfino_next_area},
    {STATE_SHELFINO_SEARCH_BLOCK, ass_3::shelfino_search_block},
    {STATE_SHELFINO_CHECK_BLOCK, ass_3::shelfino_check_block},
    {STATE_SHELFINO_PARK, ass_3::shelfino_park},
    {STATE_UR5_LOAD, ass_3::ur5_load},
    {STATE_UR5_UNLOAD, ass_3::ur5_unload},
};

/* Global variables */

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

    // ROS Node initialization
    ros::init(argc, argv, "fsm_controller");
    ros::NodeHandle fsm_node;
    ros::Rate loop_rate(100.);

    int assignment_number;
    ros::param::get("~assignment", assignment_number);
    ros::param::get("real_robot", real_robot);
    ROS_INFO("Executing assignment %d", assignment_number);
    ROS_INFO("Using real robot: %d", real_robot);
    
    // Setup services
    ur5_move_client = fsm_node.serviceClient<ur5_controller::MoveTo>("ur5/move_to");
    ur5_gripper_client = fsm_node.serviceClient<ur5_controller::SetGripper>("ur5/set_gripper");
    shelfino_move_client = fsm_node.serviceClient<shelfino_controller::MoveTo>("shelfino/move_to");
    shelfino_rotate_client = fsm_node.serviceClient<shelfino_controller::Rotate>("shelfino/rotate");
    shelfino_point_client = fsm_node.serviceClient<shelfino_controller::PointTo>("shelfino/point_to");
    shelfino_forward_client = fsm_node.serviceClient<shelfino_controller::MoveForward>("shelfino/move_forward");

    // Vision services
    detection_client = fsm_node.serviceClient<robotic_vision::Detect>("shelfino/yolo/detect");
    vision_stop_client = fsm_node.serviceClient<robotic_vision::Ping>("shelfino/yolo/stop");
    pointcloud_client = fsm_node.serviceClient<robotic_vision::PointCloud>("ur5/yolo/detect");

    // Gazebo services
    gazebo_set_state = fsm_node.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_get_state = fsm_node.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_link_attacher = fsm_node.serviceClient<gazebo_ros_link_attacher::Attach>("link_attacher_node/attach");
    gazebo_link_detacher = fsm_node.serviceClient<gazebo_ros_link_attacher::Attach>("link_attacher_node/detach");

    // Get world params
    get_world_params(fsm_node);

    // Initial state
    current_state = STATE_INIT;

    // Wait for other ROS nodes
    ur5_move_client.waitForExistence();
    shelfino_rotate_client.waitForExistence();
    shelfino_forward_client.waitForExistence();
    detection_client.waitForExistence();
    // system("clear");

    while (ros::ok())
    {
        if (current_state < STATE_END)
        {
            ROS_DEBUG("Executing state function %d", current_state);

            if (assignment_number == 1)
                (fsm_ass_1[current_state])();      
            else if (assignment_number == 2)
                (fsm_ass_2[current_state])();      
            else if (assignment_number == 3)
                (fsm_ass_3[current_state])();    
            else
                (fsm_test[current_state])();  

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
