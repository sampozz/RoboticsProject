#ifndef __KINEMATICS_TYPES_H__
#define __KINEMATICS_TYPES_H__

#include <Eigen/Dense>

/* Custom types */

/**
 * 6x1 column matrix containing the values of the joints of the UR5
 */
typedef Eigen::Matrix<double, 6, 1> JointStateVector;

/**
 * 3x1 column matrix containing the values of the joints of the gripper.
 * The first 2 rows only are used if soft gripper is enabled 
 */
typedef Eigen::Matrix<double, 3, 1> GripperStateVector;

/**
 * 6x1 column matrix containing the cartesian coordinates (x, y, z) 
 */
typedef Eigen::Matrix<double, 3, 1> Coordinates;

/**
 * 3x3 matrix containing the values of the algebraic rotation matrix 
 */
typedef Eigen::Matrix<double, 3, 3> RotationMatrix;

/**
 * 4x4 matrix containing the values of the homogeneous transformation matrix
 */
typedef Eigen::Matrix<double, 4, 4> HomoTrMatrix;

#endif