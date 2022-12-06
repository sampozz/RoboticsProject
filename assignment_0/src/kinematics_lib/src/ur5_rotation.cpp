#include "kinematics_lib/ur5_kinematics.h"

/* Public functions */

void euler_to_rot(double roll, double pitch, double yaw, RotationMatrix &rot)
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

/**
 * Thanks Wikipedia
 */
void quaternion_to_rot(double qx, double qy, double qz, double qw, RotationMatrix &rot)
{
    double roll, pitch, yaw;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    euler_to_rot(roll, pitch, yaw, rot);
}