#ifndef DLQR_HPP
#define DLQR_HPP

#include <iostream>
#include <Eigen/Dense>

/* ************************************ option start ************************************ */
/* approximate mode: saves calculation time but reduces calculation accuracy */
// #define DLQR_APPROXIMATE_MODE
/* print Ad */
#define DLQR_TEST_PRINT_Ad
/* print Bd */
#define DLQR_TEST_PRINT_Bd
/* ************************************ option end ************************************ */

enum SYSTEM_TYPE {CONTINUOUS, DISCRETE};

class DLQR
{
    private:
        /* dim of state vector */
        int dim_state;
        /* dim of control vector */
        int dim_control;
        /* sample period */
        double T;

        Eigen::MatrixXd Ad;
        Eigen::MatrixXd Bd;

    protected:
        
    public:
        DLQR(Eigen::MatrixXd A, Eigen::MatrixXd B,
             int dim_state_, int dim_control_, double T_, 
             SYSTEM_TYPE system_type_);
        ~DLQR();
};

#endif