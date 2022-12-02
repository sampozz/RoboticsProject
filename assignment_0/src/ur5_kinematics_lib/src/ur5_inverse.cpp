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
}

void ur5_inverse_complete(Coordinates &pe, RotationMatrix &re, Eigen::Matrix<double, 8, 6> &th)
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
    double th1_1 = real(atan2(p50(1, 0), p50(0, 0)) * complex_converter + acos(dh_d[3] / (sqrt(pow(p50(1, 0), 2) + pow(p50(0, 0), 2)))) * complex_converter + M_PI_2);
    double th1_2 = real(atan2(p50(1, 0), p50(0, 0)) * complex_converter - acos(dh_d[3] / (sqrt(pow(p50(1, 0), 2) + pow(p50(0, 0), 2)))) * complex_converter + M_PI_2);

    /* Finding th5 */
    double th5_1 = real(acos((pe[0] * sin(th1_1) - pe[1] * cos(th1_1) - dh_d[3]) / dh_d[5] * complex_converter));
    double th5_2 = -real(acos((pe[0] * sin(th1_1) - pe[1] * cos(th1_1) - dh_d[3]) / dh_d[5] * complex_converter));
    double th5_3 = real(acos((pe[0] * sin(th1_2) - pe[1] * cos(th1_2) - dh_d[3]) / dh_d[5] * complex_converter));
    double th5_4 = -real(acos((pe[0] * sin(th1_2) - pe[1] * cos(th1_2) - dh_d[3]) / dh_d[5] * complex_converter));

    /* Finding th6 */
    // HomoTrMatrix t06 = t60.completeOrthogonalDecomposition().pseudoInverse();
    HomoTrMatrix t06 = t60.inverse();
    Coordinates x_hat;
    x_hat << t06(0, 0), t06(1, 0), t06(2, 0);
    Coordinates y_hat;
    y_hat << t06(0, 1), t06(1, 1), t06(2, 1);
    double th6_1 = real(atan2((-x_hat(1) * sin(th1_1) + y_hat(1) * cos(th1_1)) / sin(th5_1), (x_hat(0) * sin(th1_1) - y_hat(0) * cos(th1_1)) / sin(th5_1)) * complex_converter);
    double th6_2 = real(atan2((-x_hat(1) * sin(th1_1) + y_hat(1) * cos(th1_1)) / sin(th5_2), (x_hat(0) * sin(th1_1) - y_hat(0) * cos(th1_1)) / sin(th5_2)) * complex_converter);
    double th6_3 = real(atan2((-x_hat(1) * sin(th1_2) + y_hat(1) * cos(th1_2)) / sin(th5_3), (x_hat(0) * sin(th1_2) - y_hat(0) * cos(th1_2)) / sin(th5_3)) * complex_converter);
    double th6_4 = real(atan2((-x_hat(1) * sin(th1_2) + y_hat(1) * cos(th1_2)) / sin(th5_4), (x_hat(0) * sin(th1_2) - y_hat(0) * cos(th1_2)) / sin(th5_4)) * complex_converter);

    /* Finding th3 */
    HomoTrMatrix t41m = t10f(th1_1).inverse() * t60 * t65f(th6_1).inverse() * t54f(th5_1).inverse();
    Coordinates p41_1;
    p41_1 << t41m(0, 3), t41m(1, 3), t41m(2, 3);
    double p41xz_1 = sqrt(pow(p41_1(0), 2) + pow(p41_1(2), 2));

    t41m = t10f(th1_1).inverse() * t60 * t65f(th6_2).inverse() * t54f(th5_2).inverse();
    Coordinates p41_2;
    p41_2 << t41m(0, 3), t41m(1, 3), t41m(2, 3);
    double p41xz_2 = sqrt(pow(p41_2(0), 2) + pow(p41_2(2), 2));

    t41m = t10f(th1_2).inverse() * t60 * t65f(th6_3).inverse() * t54f(th5_3).inverse();
    Coordinates p41_3;
    p41_3 << t41m(0, 3), t41m(1, 3), t41m(2, 3);
    double p41xz_3 = sqrt(pow(p41_3(0), 2) + pow(p41_3(2), 2));

    t41m = t10f(th1_2).inverse() * t60 * t65f(th6_4).inverse() * t54f(th5_4).inverse();
    Coordinates p41_4;
    p41_4 << t41m(0, 3), t41m(1, 3), t41m(2, 3);
    double p41xz_4 = sqrt(pow(p41_4(0), 2) + pow(p41_4(2), 2));

    double th3_1 = real(acos((pow(p41xz_1, 2) - pow(dh_a[1], 2) - pow(dh_a[2], 2)) / (2 * dh_a[1] * dh_a[2]) * complex_converter));
    double th3_2 = real(acos((pow(p41xz_2, 2) - pow(dh_a[1], 2) - pow(dh_a[2], 2)) / (2 * dh_a[1] * dh_a[2]) * complex_converter));
    double th3_3 = real(acos((pow(p41xz_3, 2) - pow(dh_a[1], 2) - pow(dh_a[2], 2)) / (2 * dh_a[1] * dh_a[2]) * complex_converter));
    double th3_4 = real(acos((pow(p41xz_4, 2) - pow(dh_a[1], 2) - pow(dh_a[2], 2)) / (2 * dh_a[1] * dh_a[2]) * complex_converter));

    double th3_5 = -th3_1;
    double th3_6 = -th3_2;
    double th3_7 = -th3_3;
    double th3_8 = -th3_4;

    /* Finding th2 */
    double th2_1 = real(atan2(-p41_1(2, 0), -p41_1(0, 0)) * complex_converter - asin((-dh_a[2] * sin(th3_1)) / p41xz_1 * complex_converter) * complex_converter);
    double th2_2 = real(atan2(-p41_2(2, 0), -p41_2(0, 0)) * complex_converter - asin((-dh_a[2] * sin(th3_2)) / p41xz_2 * complex_converter) * complex_converter);
    double th2_3 = real(atan2(-p41_3(2, 0), -p41_3(0, 0)) * complex_converter - asin((-dh_a[2] * sin(th3_3)) / p41xz_3 * complex_converter) * complex_converter);
    double th2_4 = real(atan2(-p41_4(2, 0), -p41_4(0, 0)) * complex_converter - asin((-dh_a[2] * sin(th3_4)) / p41xz_4 * complex_converter) * complex_converter);

    double th2_5 = real(atan2(-p41_1(2, 0), -p41_1(0, 0)) * complex_converter - asin((dh_a[2] * sin(th3_1)) / p41xz_1 * complex_converter) * complex_converter);
    double th2_6 = real(atan2(-p41_2(2, 0), -p41_2(0, 0)) * complex_converter - asin((dh_a[2] * sin(th3_2)) / p41xz_2 * complex_converter) * complex_converter);
    double th2_7 = real(atan2(-p41_3(2, 0), -p41_3(0, 0)) * complex_converter - asin((dh_a[2] * sin(th3_3)) / p41xz_3 * complex_converter) * complex_converter);
    double th2_8 = real(atan2(-p41_4(2, 0), -p41_4(0, 0)) * complex_converter - asin((dh_a[2] * sin(th3_4)) / p41xz_4 * complex_converter) * complex_converter);

    /* Finding th4 */
    HomoTrMatrix t43m = t32f(th3_1).inverse() * t21f(th2_1).inverse() * t10f(th1_1).inverse() * t60 * t65f(th6_1).inverse() * t54f(th5_1).inverse();
    Coordinates x_hat43;
    x_hat43 << t43m(0, 0), t43m(1, 0), t43m(2, 0);
    double th4_1 = real(atan2(x_hat43(1), x_hat43(0)) * complex_converter);

    t43m = t32f(th3_2).inverse() * t21f(th2_2).inverse() * t10f(th1_1).inverse() * t60 * t65f(th6_2).inverse() * t54f(th5_2).inverse();
    x_hat43 << t43m(0, 0), t43m(1, 0), t43m(2, 0);
    double th4_2 = real(atan2(x_hat43(1), x_hat43(0)) * complex_converter);

    t43m = t32f(th3_3).inverse() * t21f(th2_3).inverse() * t10f(th1_2).inverse() * t60 * t65f(th6_3).inverse() * t54f(th5_3).inverse();
    x_hat43 << t43m(0, 0), t43m(1, 0), t43m(2, 0);
    double th4_3 = real(atan2(x_hat43(1), x_hat43(0)) * complex_converter);

    t43m = t32f(th3_4).inverse() * t21f(th2_4).inverse() * t10f(th1_2).inverse() * t60 * t65f(th6_4).inverse() * t54f(th5_4).inverse();
    x_hat43 << t43m(0, 0), t43m(1, 0), t43m(2, 0);
    double th4_4 = real(atan2(x_hat43(1), x_hat43(0)) * complex_converter);

    t43m = t32f(th3_5).inverse() * t21f(th2_5).inverse() * t10f(th1_1).inverse() * t60 * t65f(th6_1).inverse() * t54f(th5_1).inverse();
    x_hat43 << t43m(0, 0), t43m(1, 0), t43m(2, 0);
    double th4_5 = real(atan2(x_hat43(1), x_hat43(0)) * complex_converter);

    t43m = t32f(th3_6).inverse() * t21f(th2_6).inverse() * t10f(th1_1).inverse() * t60 * t65f(th6_2).inverse() * t54f(th5_2).inverse();
    x_hat43 << t43m(0, 0), t43m(1, 0), t43m(2, 0);
    double th4_6 = real(atan2(x_hat43(1), x_hat43(0)) * complex_converter);

    t43m = t32f(th3_7).inverse() * t21f(th2_7).inverse() * t10f(th1_2).inverse() * t60 * t65f(th6_3).inverse() * t54f(th5_3).inverse();
    x_hat43 << t43m(0, 0), t43m(1, 0), t43m(2, 0);
    double th4_7 = real(atan2(x_hat43(1), x_hat43(0)) * complex_converter);

    t43m = t32f(th3_8).inverse() * t21f(th2_8).inverse() * t10f(th1_2).inverse() * t60 * t65f(th6_4).inverse() * t54f(th5_4).inverse();
    x_hat43 << t43m(0, 0), t43m(1, 0), t43m(2, 0);
    double th4_8 = real(atan2(x_hat43(1), x_hat43(0)) * complex_converter);

    th << th1_1, th2_1, th3_1, th4_1, th5_1, th6_1,
        th1_1, th2_2, th3_2, th4_2, th5_2, th6_2,
        th1_2, th2_3, th3_3, th4_3, th5_3, th6_3,
        th1_2, th2_4, th3_4, th4_4, th5_4, th6_4,
        th1_1, th2_5, th3_5, th4_5, th5_1, th6_1,
        th1_1, th2_6, th3_6, th4_6, th5_2, th6_2,
        th1_2, th2_7, th3_7, th4_7, th5_3, th6_3,
        th1_2, th2_8, th3_8, th4_8, th5_4, th6_4;
}