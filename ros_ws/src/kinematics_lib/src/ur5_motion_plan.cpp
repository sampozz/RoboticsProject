#include "kinematics_lib/ur5_kinematics.h"
#include <iostream>

double *ur5_trajectory_plan(const JointStateVector &initial_joints, const JointStateVector &final_joints, int n)
{
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
    double *theta = (double *)malloc(sizeof(double) * 6 * n);

    for (double t = 0; t < 1; t += 1.0 / n)
    {
        for (int i = 0; i < 6; i++)
        {
            double q = coeff(i, 0) + coeff(i, 1) * t + coeff(i, 2) * t * t + coeff(i, 3) * t * t * t;
            theta[(int)(t * n) * 6 + i] = q;
        }
    }

    return theta;
}