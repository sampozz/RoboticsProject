#ifndef __UR5_KINEMATICS_H__
#define __UR5_KINEMATICS_H__

#include <Eigen/Dense>
#include <math.h>

/* Custom types */
typedef Eigen::Matrix<double, 6, 1> JointStateVector;
typedef Eigen::Matrix<double, 6, 1> DHParameters;
typedef Eigen::Matrix<double, 3, 1> Coordinates;
typedef Eigen::Matrix<double, 3, 3> RotationMatrix;
typedef Eigen::Matrix<double, 4, 4> HomoTrMatrix;

/* UR5 parameters */
// extern DHParameters dh_a;
// extern DHParameters dh_d;
// extern DHParameters dh_alpha;

/* Functions */

/**
 * Initialize links_length, joints_dimension, joints_limit vectors with the values of the UR5 parameters
 */
void ur5_init_dh_params();

/**
 * Compute direct kinematics of UR5
 * th: input - six joints values (angles)
 * pe: output - cartesian position of the end effector
 * re: output - rotation matrix of the end effector
 */
void ur5_direct(JointStateVector &th, Coordinates &pe, RotationMatrix &re);

/**
 * Compute inverse kinematics of UR5
 * pe: input - desired cartesian position of the end effector
 * re: input - desired rotation matrix of the end effector
 * th: output - six joints values (angles)
 */
void ur5_inverse(Coordinates &pe, RotationMatrix &re, JointStateVector &th);

#endif