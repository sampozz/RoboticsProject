#ifndef __UR5_KINEMATICS_H__
#define __UR5_KINEMATICS_H__

#include <Eigen/Dense>
#include <math.h>

/* Custom types */

typedef Eigen::Matrix<double, 6, 1> JointStateVector;
typedef Eigen::Matrix<double, 3, 1> GripperStateVector;
typedef Eigen::Matrix<double, 3, 1> Coordinates;
typedef Eigen::Matrix<double, 3, 3> RotationMatrix;
typedef Eigen::Matrix<double, 4, 4> HomoTrMatrix;

/* UR5 DH parameters */

static const double dh_a[] = {0, -0.425, -0.3922, 0, 0, 0};
static const double dh_d[] = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};
static const double dh_alpha[] = {M_PI / 2, 0.0, 0.0, M_PI / 2, -M_PI / 2, 0};

/* Functions */

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

/**
 * Compute rotation matrix from euler angles.
 * rot: output - rotation matrix
 */
void euler_to_rot(double roll, double pitch, double yaw, RotationMatrix &rot);

#endif