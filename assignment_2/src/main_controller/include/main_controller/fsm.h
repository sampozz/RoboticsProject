/** 
* @file fsm.h 
* @brief Header file for the finite state machine which implements the main actions 
*
* @author Samuele Pozzani
*
* @date 18/12/2022
*/

#ifndef __FSM_H__
#define __FSM_H__

#include "ur5_controller/MoveTo.h"
#include "ur5_controller/SetGripper.h"
#include "shelfino_controller/MoveTo.h"
#include "shelfino_controller/Rotate.h"
#include "shelfino_controller/PointTo.h"
#include "shelfino_controller/MoveForward.h"
#include "yolov5_ros/Classify.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_ros_link_attacher/Attach.h"
#include <vector>
#include <map>

/* FSM */

typedef enum
{
    STATE_INIT,
    STATE_SHELFINO_ROTATE_AREA,
    STATE_SHELFINO_NEXT_AREA,
    STATE_SHELFINO_SEARCH_BLOCK,
    STATE_SHELFINO_CHECK_BLOCK,
    STATE_UR5_LOAD,
    STATE_UR5_UNLOAD,
    STATE_END
} State_t;

typedef struct
{
    State_t state;
    void (*state_function)(void);
} StateMachine_t;

State_t current_state;


/* Global variables */

ur5_controller::Coordinates ur5_home_pos, ur5_load_pos, ur5_unload_pos;
ur5_controller::EulerRotation ur5_default_rot;
shelfino_controller::Coordinates shelfino_home_pos, shelfino_current_pos;
geometry_msgs::Pose block_load_pos;

ur5_controller::MoveTo ur5_move_srv;
ur5_controller::SetGripper ur5_gripper_srv;
shelfino_controller::MoveTo shelfino_move_srv;
shelfino_controller::Rotate shelfino_rotate_srv;
shelfino_controller::PointTo shelfino_point_srv;
shelfino_controller::MoveForward shelfino_forward_srv;

yolov5_ros::Classify detection_srv;

gazebo_msgs::SetModelState model_state_srv;
gazebo_ros_link_attacher::Attach link_attacher_srv;

ros::ServiceClient ur5_move_client, ur5_gripper_client, shelfino_move_client, shelfino_point_client, detection_client,
    shelfino_rotate_client, shelfino_forward_client, gazebo_model_state, gazebo_link_attacher, gazebo_link_detacher;

std::vector<std::vector<double>> areas;
std::vector<double> unload_pos_y;
std::map<int, int> class_to_basket_map;

int current_area_index; // Index of the current area in the areas array (different to area number)
int current_block_class;
double current_block_distance;


/* States functions */

void init();
void shelfino_rotate_towards_next_area();
void shelfino_next_area();
void shelfino_search_block();
void shelfino_check_block();
void ur5_load();
void ur5_unload();

void attach();
void detach();

#endif