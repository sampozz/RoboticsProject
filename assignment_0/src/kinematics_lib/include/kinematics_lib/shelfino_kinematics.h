#ifndef __SHELFINO_KINEMATICS_H__
#define __SHELFINO_KINEMATICS_H__

#include "kinematics_lib/kinematics_types.h"

/**
 * Compute rotation matrix from quaternion
 */
void quaternion_to_rot(double qx, double qy, double qz, double qw, RotationMatrix &rot);

#endif