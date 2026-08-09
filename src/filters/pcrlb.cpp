#include "../../include/filters/pcrlb.hpp"

PCRLB::PCRLB(const Eigen::MatrixXd& init_P, double dt_){
    dt = dt_;

    Eigen::LDLT<Eigen::MatrixXd> ldlt0(init_P);
    if(ldlt0.info()!=Eigen::Success)
        throw std::runtime_error("Initial covariance decomposition failed.");
    I = ldlt0.solve(Eigen::MatrixXd::Identity(init_P.rows(), init_P.cols()));

    Eigen::MatrixXd Q = int_proc_noise_mat_Q_CV(dt);
    Eigen::LDLT<Eigen::MatrixXd> ldlt(Q);
    if(ldlt.info() != Eigen::Success)
        throw std::runtime_error("PCRLB update: Process noise matrix decomposition failed.");
    Q_inv = ldlt.solve(Eigen::MatrixXd::Identity(Q.rows(),Q.cols()));

    Eigen::Matrix2d R = int_meas_noise_mat_R_CV();
    Eigen::LDLT<Eigen::Matrix2d> ldlt2(R);
    if(ldlt2.info() != Eigen::Success)
        throw std::runtime_error("PCRLB update: Measurement noise matrix decomposition failed.");
    R_inv = ldlt2.solve(Eigen::Matrix2d::Identity());
}

Eigen::Vector2d PCRLB::convertIToRmseVector() const {
    
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(I, Eigen::ComputeThinU | Eigen::ComputeThinV);
    
    Eigen::VectorXd singular_values = svd.singularValues();
    Eigen::VectorXd singular_values_inv(singular_values.size());
    
    for (int i = 0; i < singular_values.size(); ++i) {
        if (singular_values(i) > 1e-12) {
            singular_values_inv(i) = 1.0 / singular_values(i);
        } else {
            singular_values_inv(i) = 0.0; // Hoặc để 1 giá trị penalty cực lớn
        }
    }
    
    Eigen::MatrixXd PCRLB_ = svd.matrixV() * singular_values_inv.asDiagonal() * svd.matrixU().transpose();

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(I);

    double pos_var = PCRLB_(0,0) + PCRLB_(2,2) + PCRLB_(4,4);
    double vel_var = PCRLB_(1,1) + PCRLB_(3,3) + PCRLB_(5,5);

    // Cảnh báo nếu phương sai âm (điều này không được phép xảy ra với SVD chuẩn)
    if (pos_var < 0 || vel_var < 0) {
        std::cerr << "[Cảnh báo] CRLB tính ra phương sai âm! Kiểm tra lại ma trận FIM." << std::endl;
        pos_var = std::max(0.0, pos_var);
        vel_var = std::max(0.0, vel_var);
    }

    return Eigen::Vector2d(std::sqrt(pos_var), std::sqrt(vel_var));
}

void PCRLB::update(const Eigen::VectorXd& x_true, const Eigen::Vector3d& observer_pos){
    // std::cout << "target pos : (" << x_true(0) << ", " << x_true(2) << "," <<  x_true(4) << ")\n"; 
    Eigen::MatrixXd H = computeJacobianH(x_true, observer_pos);
    
    Eigen::MatrixXd D11 = F.transpose() * Q_inv * F;
    Eigen::MatrixXd D12 = - F.transpose() * Q_inv;
    Eigen::MatrixXd D22 = Q_inv + H.transpose() * R_inv * H;

    Eigen::MatrixXd I_plus_D11 = I + D11;
    I_plus_D11.diagonal().array() += 1e-12;
    Eigen::LDLT<Eigen::MatrixXd> ldlt_tmp(I_plus_D11);
    Eigen::MatrixXd middle = ldlt_tmp.solve(Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM));
    if(ldlt_tmp.info()!=Eigen::Success)
        throw std::runtime_error("PCRLB recursion failed.");

    I = D22 - D12.transpose() * middle * D12;
    // I = 0.5*(I + I.transpose());
};