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
 * @param th input - six joints values (angles)
 * @param pe output - cartesian position of the end effector
 * @param re output - rotation matrix of the end effector
 */
void ur5_direct(JointStateVector &th, Coordinates &pe, RotationMatrix &re);

/**
 * Compute inverse kinematics of UR5, return the first solution only
 * @param pe input - desired cartesian position of the end effector
 * @param re input - desired rotation matrix of the end effector
 * @param th output - six joints values (angles)
 */
void ur5_inverse(Coordinates &pe, RotationMatrix &re, JointStateVector &th);

/**
 * Compute inverse kinematics of UR5, return all the 8 solutions in a 8x6 matrix
 * @param pe input - desired cartesian position of the end effector
 * @param re input - desired rotation matrix of the end effector
 * @param th output - 8x6 matrix containing the 8 solutions of the inverse kinematics
 */
void ur5_inverse_complete(Coordinates &pe, RotationMatrix &re, Eigen::Matrix<double, 8, 6> &th);

/**
 * Compute the jacobian matrix of the ur5 for the given configuration
 * @param th input - joints configuration
 * @param jac output - 6x6 Jacobian matrix
*/
void ur5_jacobian(JointStateVector &th, Eigen::Matrix<double, 6, 6> &jac);

/**
 * Compute rotation matrix from euler angles
 * @param roll The roll euler angle
 * @param pitch The pitch euler angle
 * @param yaw The yaw euler angle
 * @param rot output - rotation matrix
 */
void euler_to_rot(double roll, double pitch, double yaw, RotationMatrix &rot);

/**
 * Compute the joints values to follow a path between initial and final positions by finding the coefficients of a third degree system.
 * @param initial_joints Starting joints configuration  
 * @param final_joints Final joints configuration
 * @param n Number of configurations to compute in between the path
 * @return Pointer to a vector of n * 6 elements, representing the computed configurations 
 */
double *ur5_trajectory_plan(JointStateVector &initial_joints, JointStateVector &final_joints, int n);

#endif