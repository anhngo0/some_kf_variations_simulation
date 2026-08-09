#include "../../include/filters/filter_lib.hpp"

double normalize_angle(double angle) {
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

Eigen::MatrixXd int_proc_noise_mat_Q_CV(double dt){
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
    Eigen::Matrix2d Q_block;
    double dt2 = dt* dt;
    double dt3 = dt2 * dt;
    double dt4 = dt2 * dt2;

    Q_block <<
    dt4/4.0, dt3/2.0,
    dt3/2.0, dt2;

    Q_block *= ACC_STD * ACC_STD;
    Q.block<2,2>(0,0)=Q_block;
    Q.block<2,2>(2,2)=Q_block;
    Q.block<2,2>(4,4)=Q_block;
    return Q; 
}

Eigen::Matrix2d int_meas_noise_mat_R_CV(){
    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R(0, 0) = std::pow(MEAS_STD, 2); // azimuth noise
    R(1, 1) = std::pow(MEAS_STD, 2); // elevation noise
    return R;
}


Eigen::MatrixXd init_Transit_Mat_CV(double dt){
    Eigen::MatrixXd F(6,6);
    F << 1 , dt, 0 , 0, 0 ,0, 
        0,  1,  0 , 0, 0 ,0,
        0,  0,  1 , dt, 0,0,
        0,  0,  0 , 1 , 0,0,
        0,  0,  0,  0 , 1, dt,
        0,  0,  0,  0,  0, 1;
    return F;
};


/*For EKF*/
Eigen::MatrixXd computeJacobianH(const Eigen::VectorXd& state, const Eigen::Vector3d& observer_pos) 
{
    double x = state(0) - observer_pos(0);
    double y = state(2) - observer_pos(1);
    double z = state(4) - observer_pos(2);

    double p2 = x*x + y*y;
    double p  = std::sqrt(p2);
    double r2 = p2 + z*z;

    Eigen::MatrixXd H_res = Eigen::MatrixXd::Zero(2, STATE_DIM);

    constexpr double eps = 1e-6;

    if (p < eps || r2 < eps)
        return H_res;

    H_res(0,0) = -y / p2;
    H_res(0,2) =  x / p2;

    H_res(1,0) = -x*z / (p*r2);
    H_res(1,2) = -y*z / (p*r2);
    H_res(1,4) =  p   / r2;
    return H_res;
}

Eigen::VectorXd nonlinear_meas_func(const Eigen::VectorXd& state, const Eigen::Vector3d& observer_pos) 
{
    Eigen::Vector2d z_pred;

    double x = state(0) - observer_pos(0);
    double y = state(2) - observer_pos(1);
    double z = state(4) - observer_pos(2);

    double p = std::sqrt(x*x + y*y);

    z_pred(0) = std::atan2(y, x);      // azimuth
    z_pred(1) = std::atan2(z, p);      // elevation

    return z_pred;
}

