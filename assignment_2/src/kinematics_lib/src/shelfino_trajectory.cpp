#include "kinematics_lib/shelfino_kinematics.h"
#include "kinematics_lib/kinematics_types.h"
#include <iostream>

const double kp = 1;
const double kth = 1;

double shelfino_trajectory(Coordinates &initial_pos, double initial_rot, Coordinates &final_position)
{
    double dx = final_position(0) - initial_pos(0);
    double dy = final_position(1) - initial_pos(1);
    double alpha = norm_angle(atan2(dy, dx));
    double angle = norm_angle(initial_rot - alpha);

    // correggi il segno in base alla rotazione iniziale
    if (angle > M_PI)
        angle = -(2 * M_PI - angle);

    return angle;
}

double sinc(double t)
{
    double s;
    if (t == 0.0)
        s = 1.0;
    else
        s = sin(t) / t;
    return s;
}

void line_control(Coordinates &initial_pos, double initial_rot, Coordinates &desired_pos, double desired_rot, double desired_linvel, double desired_angvel, double &linear_vel, double &angular_vel)
{
    double error_x = initial_pos(0) - desired_pos(0);
    double error_y = initial_pos(1) - desired_pos(1);
    double error_rot = initial_rot - desired_rot;

    double psi = atan2(error_y, error_x);
    double alpha = (initial_rot + desired_rot) / 2;
    double error_xy = sqrt(error_x * error_x + error_y * error_y);

    double dv = -kp * error_xy * cos(psi - initial_rot);
    linear_vel = dv + desired_linvel;

    double domega = -kth * error_rot - desired_linvel * sinc(error_rot / 2) * error_xy * sin(psi - alpha);
    angular_vel = domega + desired_angvel;
}