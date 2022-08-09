#include "DLQR.hpp"

void PrintMatrix(Eigen::MatrixXd mat)
{
    for(int i = 0 ; i < mat.rows() ; i++)
    {
        for(int j = 0 ; j < mat.cols() ; j++)
        {
            std::cout << mat(i, j) << "  ";
        }
        std::cout << std::endl;
    }
}

/* constructor */
DLQR::DLQR(Eigen::MatrixXd A, Eigen::MatrixXd B, 
           int dim_state_, int dim_control_, double T_,
           SYSTEM_TYPE system_type_)
{
    dim_state = dim_state_;
    dim_control = dim_control_;
    T = T_;

    Q.resize(dim_state, dim_state);
    R.resize(dim_control, dim_control);

    /* error checking */
    if((A.rows()) != dim_state)
        std::cout << "A is in wrong rows !!!" << std::endl; 
    if((A.cols()) != dim_state)
        std::cout << "A is in wrong cols !!!" << std::endl; 
    if((B.rows()) != dim_state)
        std::cout << "B is in wrong rows !!!" << std::endl; 
    if((B.cols()) != dim_control)
        std::cout << "B is in wrong cols !!!" << std::endl; 

    /* discretization */
    if(system_type_ == CONTINUOUS)
    {
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim_state, dim_state);
        Ad = (I - 0.5 * T * A).inverse() * (I + 0.5 * T * A);
        Bd = (I - 0.5 * T * A).inverse() * B;
        #ifdef DLQR_APPROXIMATE_MODE
            Bd = B;
        #endif
    }
    else
    {
        Ad = A;
        Bd = B;
    }

    #ifdef DLQR_TEST_PRINT_Ad
        std::cout << "----------------------------------------- Ad -----------------------------------------" << std::endl;
        PrintMatrix(Ad);
    #endif
    #ifdef DLQR_TEST_PRINT_Bd
        std::cout << "----------------------------------------- Bd -----------------------------------------" << std::endl;
        PrintMatrix(Bd);
    #endif

    std::cout << "DLQR Birth Done" << std::endl;
}

/* destructor */
DLQR::~DLQR()
{
    std::cout << "DLQR Die ..." << std::endl;
}

void DLQR::DLQRInit()
{
    /* ************************************ option start ************************************ */
    /* Q init */
    Q << 100.0, 0.0,
         0.0, 100.0;
    #ifdef DLQR_TEST_PRINT_Q
        std::cout << "----------------------------------------- Q -----------------------------------------" << std::endl;
        PrintMatrix(Q);
    #endif

    /* R init */
    R << 100.0;
    #ifdef DLQR_TEST_PRINT_R
        std::cout << "----------------------------------------- R -----------------------------------------" << std::endl;
        PrintMatrix(R);
    #endif
    /* ************************************ option end ************************************ */
}

void DLQR::DLQRRun()
{
    Eigen::MatrixXd P = Q;
    Eigen::MatrixXd P_1;
    Eigen::MatrixXd P_err;
    int iteration_num = 0;
    double err = 10.0 * DLQR_TOLERANCE;
    Eigen::MatrixXd::Index maxRow, maxCol;

    while(err > DLQR_TOLERANCE && iteration_num < DLQR_MAX_ITERATION)
    {
        iteration_num++;
        P_1 = Q + Ad.transpose() * P * Ad - Ad.transpose() * P * Bd * (R + Bd.transpose() * P * Bd).inverse() * Bd.transpose() * P * Ad;
        Eigen::MatrixXd P_err = P_1 - P;
        err = fabs(P_err.maxCoeff(&maxRow, &maxCol));
        P = P_1;
    }

    if(iteration_num < DLQR_MAX_ITERATION)
    {
        K = (R + Bd.transpose() * P * Bd).inverse() * Bd.transpose() * P * Ad;
        #ifdef DLQR_TEST_PRINT_K
            std::cout << "----------------------------------------- K -----------------------------------------" << std::endl;
            PrintMatrix(K);
        #endif
    }
    else
        std::cout << "DLQR Solve failed !!!" << std::endl;
    
    #ifdef DLQR_TEST_PRINT_ITERATION
        std::cout << "----------------------------------------- ITERATION -----------------------------------------" << std::endl;
        std::cout << iteration_num << std::endl;
    #endif
    #ifdef DLQR_TEST_PRINT_ULTI_ERROR
        std::cout << "----------------------------------------- ULTI_ERROR -----------------------------------------" << std::endl;
        std::cout << err << std::endl;
    #endif
}