#ifndef __FSM_H__
#define __FSM_H__

#include "ur5_controller/MoveTo.h"
#include "ur5_controller/SetGripper.h"
#include "shelfino_controller/MoveTo.h"
#include "gazebo_ros_link_attacher/Attach.h"
#include <vector>

/* FSM */

typedef enum
{
    STATE_INIT,
    STATE_UR5_HOME,
    STATE_UR5_LOAD,
    STATE_UR5_UNLOAD,
    STATE_SHELFINO_TEST,
    STATE_END
} State_t;

typedef struct
{
    State_t state;
    void (*state_function)(void);
} StateMachine_t;

State_t current_state;


/* Global variables */

ur5_controller::Coordinates home_pos, load_pos, unload_pos;
ur5_controller::EulerRotation default_rot;

ur5_controller::MoveTo ur5_move_srv;
ur5_controller::SetGripper ur5_gripper_srv;
shelfino_controller::MoveTo shelfino_move_srv;
gazebo_ros_link_attacher::Attach link_attacher_srv;

ros::ServiceClient ur5_move_client;
ros::ServiceClient ur5_gripper_client;
ros::ServiceClient shelfino_move_client;
ros::ServiceClient gazebo_link_attacher;
ros::ServiceClient gazebo_link_detacher;

std::vector<std::vector<double>> areas;


/* States functions */

void attach();
void detach();

void init();
void ur5_homing();
void ur5_load();
void ur5_unload();
void shelfino_test();

#endif