/** 
* @file ur5_kinematics.h 
* @brief Header file for the kinematics function related to UR5 control
*
* @author Samuele Pozzani
*
* @date 18/12/2022
*/

#ifndef __UR5_KINEMATICS_H__
#define __UR5_KINEMATICS_H__

#include <math.h>
#include "kinematics_types.h"

/* UR5 DH parameters */

static const double dh_a[] = {0, -0.425, -0.3922, 0, 0, 0};
static const double dh_d[] = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};

/* Functions */

/**
 * Compute direct kinematics of UR5
 * 
 * @param th input - six joints values (angles)
 * @param pe output - cartesian position of the end effector
 * @param re output - rotation matrix of the end effector
 */
void ur5_direct(JointStateVector &th, Coordinates &pe, RotationMatrix &re);

/**
 * Compute inverse kinematics of UR5, return the first solution only
 * 
 * @param pe The desired cartesian position of the end effector
 * @param re The desired rotation matrix of the end effector
 * @return The six joints values (angles)
 */
JointStateVector ur5_inverse(const Coordinates &pe, const RotationMatrix &re);

/**
 * Compute inverse kinematics of UR5, return all the 8 solutions in a 8x6 matrix
 * 
 * @param pe The desired cartesian position of the end effector
 * @param re The desired rotation matrix of the end effector
 * @return An 8x6 matrix containing the 8 solutions of the inverse kinematics
 */
Eigen::Matrix<double, 8, 6> ur5_inverse_complete(const Coordinates &pe, const RotationMatrix &re);

/**
 * Compute the jacobian matrix of the ur5 for the given configuration
 * 
 * @param th The joints configuration
 * @return The 6x6 Jacobian matrix
*/
Eigen::Matrix<double, 6, 6> ur5_jacobian(const JointStateVector &th);

/**
 * Compute rotation matrix from euler angles
 * 
 * @param roll The roll euler angle
 * @param pitch The pitch euler angle
 * @param yaw The yaw euler angle
 * @return The rotation matrix
 */
RotationMatrix euler_to_rot(double roll, double pitch, double yaw);

/**
 * Compute the joints values to follow a path between initial and final positions by finding the coefficients of a third degree system.
 * 
 * @param initial_joints Starting joints configuration  
 * @param final_joints Final joints configuration
 * @param n Number of configurations to compute in between the path
 * @return Pointer to a vector of n * 6 elements, representing the computed configurations 
 */
double *ur5_trajectory_plan(const JointStateVector &initial_joints, const JointStateVector &final_joints, int n);

#endif