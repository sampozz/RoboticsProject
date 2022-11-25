#include "ur5_kinematics_lib/ur5_kinematics.h"

/* Public functions */

void euler_to_rot(double roll, double pitch, double yaw, RotationMatrix& rot)
{
    RotationMatrix x_rot;
    RotationMatrix y_rot;
    RotationMatrix z_rot;

    x_rot << 1, 0, 0,
        0, cos(yaw), -sin(yaw),
        0, sin(yaw), cos(yaw);

    y_rot << cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch);

    z_rot << cos(roll), -sin(roll), 0,
        sin(roll), cos(roll), 0,
        0, 0, 1;

    rot = z_rot * y_rot * x_rot;
}