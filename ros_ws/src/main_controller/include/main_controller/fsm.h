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

#include "ros/ros.h"
#include "ur5_controller/MoveTo.h"
#include "ur5_controller/SetGripper.h"
#include "shelfino_controller/MoveTo.h"
#include "shelfino_controller/Rotate.h"
#include "shelfino_controller/PointTo.h"
#include "shelfino_controller/MoveForward.h"
#include "robotic_vision/Detect.h"
#include "robotic_vision/Stop.h"
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

/* States functions for Assignment 2 */

namespace ass_1
{
    void init(void);
    void shelfino_rotate_towards_next_area(void);
    void shelfino_next_area(void);
    void shelfino_search_block(void);
    void shelfino_check_block(void);
}

namespace ass_2 
{
    void init(void);
    void shelfino_rotate_towards_next_area(void);
    void shelfino_next_area(void);
    void shelfino_search_block(void);
    void shelfino_check_block(void);
    void ur5_load(void);
    void ur5_unload(void);
}

namespace ass_3
{
    
}

/* Utils */

void shelfino_move_to(double x, double y, double yaw);
void shelfino_forward(double distance, bool control);
void shelfino_rotate(double angle);
void shelfino_point_to(double x, double y);
void ur5_move(ur5_controller::Coordinates& pos, ur5_controller::EulerRotation& rot);
void ur5_grip(double diameter);

void attach(int model);
void detach(int model);

#endif