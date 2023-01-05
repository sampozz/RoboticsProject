/** 
* @file ur5_services.h 
* @brief Header file for the UR5 Controller Node services
*
* @author Samuele Pozzani
*
* @date 05/01/2023
*/

#ifndef __UR5_SERVICES__
#define __UR5_SERVICES__

#include "ros/ros.h"
#include "ur5_controller/ur5_controller_lib.h"
#include "ur5_controller/MoveTo.h"
#include "ur5_controller/SetGripper.h"

/**
 * Handle requests from ur5/move_to ROS service. Convert euler angles to rotation matrix
 * and call move_to function on UR5 controller.
 * 
 * @param req The service request, contains final position and rotation of the end effector
 * @param res The service response, contains result of the move_to function as status 
 */
bool srv_move_to(ur5_controller::MoveTo::Request &req, ur5_controller::MoveTo::Response &res);

/**
 * Handle requests from ur5/set_gripper ROS service. Call set_gripper function on UR5 controller.
 * 
 * @param req The service request, contains the diameter desired for the gripper
 * @param res The service response, it is empty 
 */
bool srv_set_gripper(ur5_controller::SetGripper::Request &req, ur5_controller::SetGripper::Response &res);

#endif