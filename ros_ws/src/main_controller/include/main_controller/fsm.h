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
#include "robotic_vision/Ping.h"
#include "robotic_vision/PointCloud.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h"
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
    STATE_SHELFINO_PARK,
    STATE_UR5_LOAD,
    STATE_UR5_UNLOAD,
    STATE_END
} State_t;

/**
 * Type definition for a pointer to state procedure into the finite state machine 
 */
typedef void (*state_function)(void);

/**
 * Type definition for the finite state machine
 * Map StateNumber - Pointer to StateFunction
 */
typedef std::map<int, state_function> StateMachine_t;

/** 
 * State functions for the three assignments.
 * For a description of the state functions, please refer to the project report.
 */

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
    void init(void);
    void shelfino_rotate_towards_next_area(void);
    void shelfino_next_area(void);
    void shelfino_search_block(void);
    void shelfino_check_block(void);
    void shelfino_park(void);
    void ur5_load(void);
    void ur5_unload(void);    
}

/* Utils */

/**
 * Send request to shelfino service move_to.
 * Update final position of shelfino
 * 
 * @param x The x coordinate
 * @param y The y coordinate
 * @param yaw The final rotation. If yaw == 0 do not rotate
*/
void shelfino_move_to(double x, double y, double yaw);

/**
 * Send request to shelfino service forward.
 * Update final position of shelfino
 * 
 * @param distance The distance shelfino should run
 * @param control Enables Lyapunov control on straight movement
 */
void shelfino_forward(double distance, bool control);

/**
 * Send request to shelfino service rotate
 * Update final rotation of shelfino
 * 
 * @param angle The rotation angle (negative for clockwise rotation)
 */
void shelfino_rotate(double angle);

/**
 * Send request to shelfino service point_to
 * Update final rotation of shelfino
 * 
 * @param x The x coordinate which shelfino should look
 * @param y The y coordinate which shelifno should look
*/
void shelfino_point_to(double x, double y);

/**
 * Send request to UR5 service move_to.
 * 
 * @param pos The final desired position of the end-effector
 * @param rot The final desired rotation of the end-effector
 * @return true if the movement was completed
 */
bool ur5_move(ur5_controller::Coordinates& pos, ur5_controller::EulerRotation& rot);

/**
 * Send request to UR5 service set_gripper.
 * 
 * @param diameter The desired diameter of the gripper
 */
void ur5_grip(double diameter);

/**
 * Send request to vision node service for shelfino camera.
 * 
 * @return true if a block is detected
 */
bool shelfino_detect(void);

void attach(int model, bool gripper);
void detach(int model, bool gripper);
void state_test(void);

#endif