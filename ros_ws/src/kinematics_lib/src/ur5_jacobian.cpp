#include "kinematics_lib/ur5_kinematics.h"
#include "kinematics_lib/shelfino_kinematics.h"

/* Public functions */

Eigen::Matrix<double, 6, 6> ur5_jacobian(const JointStateVector &th)
{
    Eigen::Matrix<double, 6, 6> jac;

    jac(0, 0) = dh_d[4] * (cos(th(0)) * cos(th(4)) + cos(th(1) + th(2) + th(3)) * sin(th(0)) * sin(th(4))) + dh_d[2] * cos(th(0)) + dh_d[3] * cos(th(0)) - dh_a[2] * cos(th(1) + th(2)) * sin(th(0)) - dh_a[1] * cos(th(1)) * sin(th(0)) - dh_d[4] * sin(th(1) + th(2) + th(3)) * sin(th(0));
    jac(1, 0) = dh_d[4] * (cos(th(4)) * sin(th(0)) - cos(th(1) + th(2) + th(3)) * cos(th(0)) * sin(th(4))) + dh_d[2] * sin(th(0)) + dh_d[3] * sin(th(0)) + dh_a[2] * cos(th(1) + th(2)) * cos(th(0)) + dh_a[1] * cos(th(0)) * cos(th(1)) + dh_d[4] * sin(th(1) + th(2) + th(3)) * cos(th(0));
    jac(2, 0) = 0;
    jac(3, 0) = 0;
    jac(4, 0) = 0;
    jac(5, 0) = 1;

    jac(0, 1) = -cos(th(0)) * (dh_a[2] * sin(th(1) + th(2)) + dh_a[1] * sin(th(1)) + dh_d[4] * (sin(th(1) + th(2)) * sin(th(3)) - cos(th(1) + th(2)) * cos(th(3))) - dh_d[4] * sin(th(4)) * (cos(th(1) + th(2)) * sin(th(3)) + sin(th(1) + th(2)) * cos(th(3))));
    jac(1, 1) = -sin(th(0)) * (dh_a[2] * sin(th(1) + th(2)) + dh_a[1] * sin(th(1)) + dh_d[4] * (sin(th(1) + th(2)) * sin(th(3)) - cos(th(1) + th(2)) * cos(th(3))) - dh_d[4] * sin(th(4)) * (cos(th(1) + th(2)) * sin(th(3)) + sin(th(1) + th(2)) * cos(th(3))));
    jac(2, 1) = dh_a[2] * cos(th(1) + th(2)) - (dh_d[4] * sin(th(1) + th(2) + th(3) + th(4))) / 2 + dh_a[1] * cos(th(1)) + (dh_d[4] * sin(th(1) + th(2) + th(3) - th(4))) / 2 + dh_d[4] * sin(th(1) + th(2) + th(3));
    jac(3, 1) = sin(th(0));
    jac(4, 1) = -cos(th(0));
    jac(5, 1) = 0;

    jac(0, 2) = cos(th(0)) * (dh_d[4] * cos(th(1) + th(2) + th(3)) - dh_a[2] * sin(th(1) + th(2)) + dh_d[4] * sin(th(1) + th(2) + th(3)) * sin(th(4)));
    jac(1, 2) = sin(th(0)) * (dh_d[4] * cos(th(1) + th(2) + th(3)) - dh_a[2] * sin(th(1) + th(2)) + dh_d[4] * sin(th(1) + th(2) + th(3)) * sin(th(4)));
    jac(2, 2) = dh_a[2] * cos(th(1) + th(2)) - (dh_d[4] * sin(th(1) + th(2) + th(3) + th(4))) / 2 + (dh_d[4] * sin(th(1) + th(2) + th(3) - th(4))) / 2 + dh_d[4] * sin(th(1) + th(2) + th(3));
    jac(3, 2) = sin(th(0));
    jac(4, 2) = -cos(th(0));
    jac(5, 2) = 0;

    jac(0, 3) = dh_d[4] * cos(th(0)) * (cos(th(1) + th(2) + th(3)) + sin(th(1) + th(2) + th(3)) * sin(th(4)));
    jac(1, 3) = dh_d[4] * sin(th(0)) * (cos(th(1) + th(2) + th(3)) + sin(th(1) + th(2) + th(3)) * sin(th(4)));
    jac(2, 3) = dh_d[4] * (sin(th(1) + th(2) + th(3) - th(4)) / 2 + sin(th(1) + th(2) + th(3)) - sin(th(1) + th(2) + th(3) + th(4)) / 2);
    jac(3, 3) = sin(th(0));
    jac(4, 3) = -cos(th(0));
    jac(5, 3) = 0;

    jac(0, 4) = -dh_d[4] * sin(th(0)) * sin(th(4)) - dh_d[4] * cos(th(1) + th(2) + th(3)) * cos(th(0)) * cos(th(4));
    jac(1, 4) = dh_d[4] * cos(th(0)) * sin(th(4)) - dh_d[4] * cos(th(1) + th(2) + th(3)) * cos(th(4)) * sin(th(0));
    jac(2, 4) = -dh_d[4] * (sin(th(1) + th(2) + th(3) - th(4)) / 2 + sin(th(1) + th(2) + th(3) + th(4)) / 2);
    jac(3, 4) = sin(th(1) + th(2) + th(3)) * cos(th(0));
    jac(4, 4) = sin(th(1) + th(2) + th(3)) * sin(th(0));
    jac(5, 4) = -cos(th(1) + th(2) + th(3));

    jac(0, 5) = 0;
    jac(1, 5) = 0;
    jac(2, 5) = 0;
    jac(3, 5) = cos(th(4)) * sin(th(0)) - cos(th(1) + th(2) + th(3)) * cos(th(0)) * sin(th(4));
    jac(4, 5) = -cos(th(0)) * cos(th(4)) - cos(th(1) + th(2) + th(3)) * sin(th(0)) * sin(th(4));
    jac(5, 5) = -sin(th(1) + th(2) + th(3)) * sin(th(4));

    return jac;
}