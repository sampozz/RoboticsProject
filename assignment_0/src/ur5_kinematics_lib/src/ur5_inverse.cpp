#include "ur5_kinematics_lib/ur5_kinematics.h"
#include <complex.h>
#include <iostream>

using namespace Eigen;
using namespace std;

/* Public functions */

void ur5_inverse(Coordinates &pe, RotationMatrix &re, JointStateVector &th)
{
    std::complex<double> complex_converter(1.0, 0.0);

    HomoTrMatrix t60;
    t60 << re(0), re(3), re(6), pe(0),
        re(1), re(4), re(7), pe(1),
        re(2), re(5), re(8), pe(2),
        0, 0, 0, 1;

    auto t10f = [&](double th_n) -> HomoTrMatrix
    { HomoTrMatrix ret; ret << cos(th_n), -sin(th_n), 0, 0,
        sin(th_n), cos(th_n), 0, 0,
        0, 0, 1, dh_d[0],
        0, 0, 0, 1; 
        return ret; };

    auto t21f = [&](double th_n) -> HomoTrMatrix
    { HomoTrMatrix ret; ret << cos(th_n), -sin(th_n), 0, 0,
        0, 0, -1, 0,
        sin(th_n), cos(th_n), 0, 0,
        0, 0, 0, 1; 
        return ret; };

    auto t32f = [&](double th_n) -> HomoTrMatrix
    { HomoTrMatrix ret; ret << cos(th_n), -sin(th_n), 0, dh_a[1],
        sin(th_n), cos(th_n), 0, 0,
        0, 0, 1, dh_d[2],
        0, 0, 0, 1; 
        return ret; };

    auto t43f = [&](double th_n) -> HomoTrMatrix
    { HomoTrMatrix ret; ret << cos(th_n), -sin(th_n), 0, dh_a[2],
        sin(th_n), cos(th_n), 0, 0,
        0, 0, 1, dh_d[3],
        0, 0, 0, 1; 
        return ret; };

    auto t54f = [&](double th_n) -> HomoTrMatrix
    { HomoTrMatrix ret; ret << cos(th_n), -sin(th_n), 0, 0,
        0, 0, -1, -dh_d[4],
        sin(th_n), cos(th_n), 0, 0,
        0, 0, 0, 1; 
        return ret; };

    auto t65f = [&](double th_n) -> HomoTrMatrix
    { HomoTrMatrix ret; ret << cos(th_n), -sin(th_n), 0, 0,
        0, 0, 1, dh_d[5],
        -sin(th_n), -cos(th_n), 0, 0,
        0, 0, 0, 1; 
        return ret; };

    /* Finding th1 */
    Eigen::Matrix<double, 4, 1> p50;
    Eigen::Matrix<double, 4, 1> c;
    c << 0.0, 0.0, -dh_d[5], 1.0;
    p50 = t60 * c;
    th(0) = real(atan2(p50(1, 0), p50(0, 0)) * complex_converter + acos(dh_d[3] / (sqrt(pow(p50(1, 0), 2) + pow(p50(0, 0), 2)))) * complex_converter + M_PI_2);

    /* Finding th5 */
    th(4) = real(acos((pe[0] * sin(th(0)) - pe[1] * cos(th(0)) - dh_d[3]) / dh_d[5] * complex_converter));

    /* Finding th6 */
    // HomoTrMatrix t06 = t60.completeOrthogonalDecomposition().pseudoInverse();
    HomoTrMatrix t06 = t60.inverse();
    Coordinates x_hat;
    x_hat << t06(0, 0), t06(1, 0), t06(2, 0);
    Coordinates y_hat;
    y_hat << t06(0, 1), t06(1, 1), t06(2, 1);
    th(5) = real(atan2((-x_hat(1) * sin(th(0)) + y_hat(1) * cos(th(0))) / sin(th(4)), (x_hat(0) * sin(th(0)) - y_hat(0) * cos(th(0))) / sin(th(4))) * complex_converter);

    /* Finding th3 */
    HomoTrMatrix t41m = t10f(th(0)).inverse() * t60 * t65f(th(5)).inverse() * t54f(th(4)).inverse();
    Coordinates p41_1;
    p41_1 << t41m(0, 3), t41m(1, 3), t41m(2, 3);
    double p41xz_1 = sqrt(pow(p41_1(0), 2) + pow(p41_1(2), 2));
    th(2) = real(acos((pow(p41xz_1, 2) - pow(dh_a[1], 2) - pow(dh_a[2], 2)) / (2 * dh_a[1] * dh_a[2]) * complex_converter));

    /* Finding th2 */
    th(1) = real(atan2(-p41_1(2, 0), -p41_1(0, 0)) * complex_converter - asin((-dh_a[2] * sin(th(2))) / p41xz_1 * complex_converter) * complex_converter);

    /* Finding th4 */
    HomoTrMatrix t43m = t32f(th(2)).inverse() * t21f(th(1)).inverse() * t10f(th(0)).inverse() * t60 * t65f(th(5)).inverse() * t54f(th(4)).inverse();
    Coordinates x_hat43;
    x_hat43 << t43m(0, 0), t43m(1, 0), t43m(2, 0);
    th(3) = real(atan2(x_hat43(1), x_hat43(0)) * complex_converter);

    for (int i = 0; i < 6; i++) 
        th(i) = fmod(th(i), 2 * M_PI);
}