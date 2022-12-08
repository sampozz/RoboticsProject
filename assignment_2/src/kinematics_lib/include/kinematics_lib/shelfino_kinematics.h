#ifndef __SHELFINO_KINEMATICS_H__
#define __SHELFINO_KINEMATICS_H__

#include "kinematics_lib/kinematics_types.h"

/**
 * Compute euler angles from quaternion
 * @return yaw
 */
double quaternion_to_yaw(double qx, double qy, double qz, double qw);

/**
 * Compute distance and required rotation to move shelfino from an initial to a final point
 * @param initial_pos input
 * @param initial_rot input
 * @param final_pos input
 * @param distance output: distance from the initial to final point
 * @param angle output: rotation angle to look towards the final point
 */
double shelfino_trajectory(Coordinates &initial_pos, double initial_rot, Coordinates &final_position);

/**
 * Return the angle between 0 and 2pi
 */
double norm_angle(double angle);

void line_control(Coordinates &initial_pos, double initial_rot, Coordinates &desired_pos, double desired_rot, double desired_linvel, double desired_angvel, double &linear_vel, double &angular_vel);

#endif