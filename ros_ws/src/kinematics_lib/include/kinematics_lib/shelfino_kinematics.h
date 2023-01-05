/** 
* @file shelfino_kinematics.h 
* @brief Header file for the kinematics functions related to Shelfino control
*
* @author Samuele Pozzani
*
* @date 18/12/2022
*/

#ifndef __SHELFINO_KINEMATICS_H__
#define __SHELFINO_KINEMATICS_H__

#include "kinematics_lib/kinematics_types.h"

/**
 * Compute euler angles from quaternion
 * @param qx Quaternion value x
 * @param qy Quaternion value y
 * @param qz Quaternion value z
 * @param qw Quaternion value w
 * @return yaw
 */
double quaternion_to_yaw(double qx, double qy, double qz, double qw);

/**
 * Compute the required rotation to move shelfino from an initial to a final point
 * 
 * @param initial_pos The initial position of Shelfino
 * @param initial_rot The initial rotation of Shelfino
 * @param final_pos The desired position of Shelfino
 * @return angle: rotation angle to look towards the final point
 */
double shelfino_trajectory_rotation(const Coordinates &initial_pos, double initial_rot, const Coordinates &final_position);

/**
 * Return the angle mapped between -pi and pi
 * 
 * @param angle The angle to be normalized
 * @return The normalized angle between -pi and pi
 */
double norm_angle(double angle);

/**
 * Compute the Lyapunov control algorithm to make Shelfino follow the desired trajectory
 * 
 * @param initial_pos The initial position
 * @param initial_rot The initial rotation
 * @param desired_pos The desired position
 * @param desired_rot The desired rotation
 * @param desired_linvel The linear velocity
 * @param desired_angvel The angular velocity
 * @param linear_vel - output: The computed linear velocity
 * @param angular_vel - output: The computed angular velocity 
 */
void line_control(const Coordinates &initial_pos, double initial_rot, const Coordinates &desired_pos, double desired_rot, double desired_linvel, double desired_angvel, double &linear_vel, double &angular_vel);

#endif