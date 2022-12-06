#ifndef __KINEMATICS_TYPES_H__
#define __KINEMATICS_TYPES_H__

#include <Eigen/Dense>

/* Custom types */

typedef Eigen::Matrix<double, 6, 1> JointStateVector;
typedef Eigen::Matrix<double, 3, 1> GripperStateVector;
typedef Eigen::Matrix<double, 3, 1> Coordinates;
typedef Eigen::Matrix<double, 3, 3> RotationMatrix;
typedef Eigen::Matrix<double, 4, 4> HomoTrMatrix;
typedef Eigen::Matrix<double, 3, 1> Velocity;

#endif