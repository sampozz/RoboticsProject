/** 
* @file shelfino_services.h 
* @brief Header file for the Shelfino Controller Node services
*
* @author Samuele Pozzani
*
* @date 05/01/2023
*/

#ifndef __SHELFINO_SERVICES_H__
#define __SHELFINO_SERVICES_H__

#include "ros/ros.h"
#include "shelfino_controller/shelfino_controller_lib.h"
#include "shelfino_controller/MoveTo.h"
#include "shelfino_controller/Rotate.h"
#include "shelfino_controller/PointTo.h"
#include "shelfino_controller/MoveForward.h"
#include "std_srvs/SetBool.h"
#include <signal.h>

/**
 * Handle requests from shelfino/move_to ROS service. Call move_to function on Shelfino controller.
 * If yaw param is equal to zero, no rotation is applied at the end of the forward movement.
 * 
 * @param req The service request, contains the coordinates of the desired position and rotation of shelfino
 * @param res The service response, contains the final rotation of shelfino
 */
bool move_to(shelfino_controller::MoveTo::Request &req, shelfino_controller::MoveTo::Response &res);

/**
 * Handle requests from shelfino/rotate ROS service. Call rotate function on Shelfino controller.
 * The rotation may be interrupted if a block is detected on the vision topic.
 * 
 * @param req The service request, contains desired angle of rotation
 * @param res The service response, contains the final rotation of shelfino
 */
bool rotate(shelfino_controller::Rotate::Request &req, shelfino_controller::Rotate::Response &res);

/**
 * Handle requests from shelfino/point_to ROS service. Call point_to function on Shelfino controller.
 * The rotation may be interrupted if a block is detected on the vision topic.
 * 
 * @param req The service request, contains the coordinates of the desired position 
 * @param res The service response, contains the final rotation of shelfino
 */
bool point_to(shelfino_controller::PointTo::Request &req, shelfino_controller::PointTo::Response &res);

/**
 * Handle requests from shelfino/move_forward ROS service. Call move_forward function on Shelfino controller.
 * The movement may be interrupted if a block is detected on the vision topic.
 * 
 * @param req The service request, contains the distance that shelfino should run
 * @param res The service response, contains the final position of shelfino
 */
bool move_forward(shelfino_controller::MoveForward::Request &req, shelfino_controller::MoveForward::Response &res);

/**
 * Signal handler to poweroff shelfino engines on CTRL+C 
 */
void handler(int sig);

#endif