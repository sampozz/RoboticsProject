#include "kinematics_lib/ur5_kinematics.h"

DHParameters dh_a;
DHParameters dh_d;
DHParameters dh_alpha;

/* Public functions */

void ur5_init_dh_params()
{
    // dh_a << 0.0, -0.425, -0.3922, 0.0, 0.0, 0.0;
    // dh_d << 0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823;
    // dh_alpha << M_PI / 2, 0.0, 0.0, M_PI / 2, -M_PI / 2, 0;
}

void ur5_direct(JointStateVector &th, Coordinates &pe, RotationMatrix &re)
{
    DHParameters dh_a;
    DHParameters dh_d;
    DHParameters dh_alpha;
    dh_a << 0, -0.425, -0.3922, 0, 0, 0;
    dh_d << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;
    dh_alpha << M_PI / 2, 0.0, 0.0, M_PI / 2, -M_PI / 2, 0;
    
    HomoTrMatrix t10m;
    HomoTrMatrix t21m;
    HomoTrMatrix t32m;
    HomoTrMatrix t43m;
    HomoTrMatrix t54m;
    HomoTrMatrix t65m;
    HomoTrMatrix t06;

    t10m << cos(th[0]), -sin(th[0]), 0, 0,
        sin(th[0]), cos(th[0]), 0, 0,
        0, 0, 1, dh_d[0],
        0, 0, 0, 1;

    t21m << cos(th[1]), -sin(th[1]), 0, 0,
        0, 0, -1, 0,
        sin(th[1]), cos(th[1]), 0, 0,
        0, 0, 0, 1;

    t32m << cos(th[2]), -sin(th[2]), 0, dh_a[1],
        sin(th[2]), cos(th[2]), 0, 0,
        0, 0, 1, dh_d[2],
        0, 0, 0, 1;

    t43m << cos(th[3]), -sin(th[3]), 0, dh_a[2],
        sin(th[3]), cos(th[3]), 0, 0,
        0, 0, 1, dh_d[3],
        0, 0, 0, 1;

    t54m << cos(th[4]), -sin(th[4]), 0, 0,
        0, 0, -1, -dh_d[4],
        sin(th[4]), cos(th[4]), 0, 0,
        0, 0, 0, 1;

    t65m << cos(th[5]), -sin(th[5]), 0, 0,
        0, 0, 1, dh_d[5],
        -sin(th[5]), -cos(th[5]), 0, 0,
        0, 0, 0, 1;

    t06 = t10m * t21m * t32m * t43m * t54m * t65m;

    pe << t06(0, 3), t06(1, 3), t06(2, 3);

    re << t06(0, 0), t06(0, 1), t06(0, 2), 
        t06(1, 0), t06(1, 1), t06(1, 2), 
        t06(2, 0), t06(2, 1), t06(2, 2);   
}

