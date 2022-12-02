#include "ur5_kinematics_lib/ur5_kinematics.h"
#include <iostream>

double *ur5_motion_plan(Coordinates &initial_pos, RotationMatrix &initial_rot, Coordinates &final_pos, RotationMatrix &final_rot, double dt)
{
    // Compute initial and final joints values
    JointStateVector initial_joints, final_joints;
    ur5_inverse(initial_pos, initial_rot, initial_joints);
    ur5_inverse(final_pos, final_rot, final_joints);

    Eigen::Matrix<double, 6, 4> coeff;
    for (int i = 0; i < 6; i++)
    {
        // Matrix associated to the third degree system
        Eigen::Matrix<double, 4, 4> m;
        m << 1, 0, 0, 0,
            0, 1, 0, 0,
            1, 1, 1, 1,
            0, 1, 2, 3;

        Eigen::Matrix<double, 4, 1> a, b;
        b << initial_joints(i), 0, final_joints(i), 0;

        // Solve system m*coeff=b
        a = m.inverse() * b;

        for (int j = 0; j < 4; j++)
            coeff(i, j) = a(j);
    }

    // Allocate dynamically the return vector
    double *theta = (double *)malloc(sizeof(double) * 6.0 / dt);

    for (double t = 0; t < 1; t += dt)
    {
        for (int i = 0; i < 6; i++)
        {
            double q = coeff(i, 0) + coeff(i, 1) * t + coeff(i, 2) * t * t + coeff(i, 3) * t * t * t;
            theta[(int)(t / dt) * 6 + i] = q;
        }
    }

    return theta;
}