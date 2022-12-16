#ifndef __FSM_H__
#define __FSM_H__

#include "ur5_controller/MoveTo.h"
#include "ur5_controller/SetGripper.h"
#include "shelfino_controller/MoveTo.h"
#include "shelfino_controller/Rotate.h"
#include "shelfino_controller/MoveForward.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_ros_link_attacher/Attach.h"
#include <vector>

/* FSM */

typedef enum
{
    STATE_INIT,
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

ur5_controller::Coordinates ur5_home_pos, ur5_load_pos, ur5_unload_pos, block_load_pos;
ur5_controller::EulerRotation ur5_default_rot;
shelfino_controller::Coordinates shelfino_home_pos;

ur5_controller::MoveTo ur5_move_srv;
ur5_controller::SetGripper ur5_gripper_srv;
shelfino_controller::MoveTo shelfino_move_srv;
shelfino_controller::Rotate shelfino_rotate_srv;
shelfino_controller::MoveForward shelfino_forward_srv;

gazebo_msgs::SetModelState model_state_srv;
gazebo_ros_link_attacher::Attach link_attacher_srv;

ros::ServiceClient ur5_move_client, ur5_gripper_client, shelfino_move_client, 
    shelfino_rotate_client, shelfino_forward_client, gazebo_model_state, gazebo_link_attacher, gazebo_link_detacher;

std::vector<std::vector<double>> areas;
int current_area;
int current_block_class;
int current_block_distance;


/* States functions */

void init();
void shelfino_next_area();
void shelfino_search_block();
void shelfino_check_block();
void ur5_load();
void ur5_unload();

void attach();
void detach();

#endif