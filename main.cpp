#include <iostream>
#include "DLQR.hpp"

int main()
{
     double T = 0.5;
     Eigen::MatrixXd x_cur;
     x_cur.resize(2, 1);
     x_cur << 2.0,
              0.0;
     Eigen::MatrixXd A;
     A.resize(2, 2);
     A << 1.0, T,
          0.0, 1.0;
     Eigen::MatrixXd B;
     B.resize(2, 1);
     B << 0.5 * T * T,
          T;
    
     DLQR *dlqr = new DLQR(A, B, 2, 1, T, DISCRETE);

     std::cout << "==================================" << std::endl;
     std::cout << x_cur(0) << "    " << x_cur(1) << std::endl;
     int cnt = 20;
     while(cnt > 0)
     {
          dlqr->DLQRInit();
          dlqr->DLQRRun();

          Eigen::MatrixXd u = -dlqr->K * x_cur;

          x_cur = dlqr->Ad * x_cur + dlqr->Bd * u;

          std::cout << x_cur(0) << "    " << x_cur(1) << std::endl;

          cnt--;
     }

     delete dlqr;
     return 0;
}