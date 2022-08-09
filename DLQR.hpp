#ifndef DLQR_HPP
#define DLQR_HPP

#include <iostream>
#include <Eigen/Dense>
#include <cmath>

/* ************************************ option start ************************************ */
/* approximate mode: saves calculation time but reduces calculation accuracy */
// #define DLQR_APPROXIMATE_MODE
/* print Ad */
// #define DLQR_TEST_PRINT_Ad
/* print Bd */
// #define DLQR_TEST_PRINT_Bd
/* print Q */
// #define DLQR_TEST_PRINT_Q
/* print R */
// #define DLQR_TEST_PRINT_R
/* print K */
// #define DLQR_TEST_PRINT_K
/* print number of iteration in solving Riccati */
// #define DLQR_TEST_PRINT_ITERATION
/* print number of ulti error in solving Riccati */
// #define DLQR_TEST_PRINT_ULTI_ERROR

/* number of max iteration in Ricatti solving */
#define DLQR_MAX_ITERATION 50000
/* number of error tolerance in Riccati solving */
#define DLQR_TOLERANCE 0.01
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
        
        /* matrix Q */
        Eigen::MatrixXd Q;
        /* matrix R */
        Eigen::MatrixXd R;
        
    protected:
        
    public:
        /* Ad matrix in discrete system */
        Eigen::MatrixXd Ad;
        /* Bd matrix in discrete system */
        Eigen::MatrixXd Bd;
        /* matrix K */
        Eigen::MatrixXd K;

        DLQR(Eigen::MatrixXd A, Eigen::MatrixXd B,
             int dim_state_, int dim_control_, double T_, 
             SYSTEM_TYPE system_type_);
        ~DLQR();
        void DLQRInit();
        void DLQRRun();
};

#endif