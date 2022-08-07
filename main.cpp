#include <iostream>
#include "DLQR.hpp"

int main()
{
    Eigen::MatrixXd A;
    A.resize(4, 4);
    A << 0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0,
         0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0;
    Eigen::MatrixXd B;
    B.resize(4, 2);
    B << 0.0, 0.0,
         0.0, 0.0,
         1.0, 0.0,
         0.0, 1.0;
    double T = 0.005;

    DLQR *dlqr = new DLQR(A, B, 4, 2, T, CONTINUOUS);

    delete dlqr;
    return 0;
}