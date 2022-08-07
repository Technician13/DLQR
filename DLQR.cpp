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