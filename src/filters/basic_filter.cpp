#include "../../include/filters/basic_filter.hpp"

void KF::predict(){
    x = F * x;
    P = F * P * F.transpose() + Q;   
};

void KF::update(const Eigen::VectorXd& meas,const Eigen::Vector3d& observer_pos){
    Eigen::VectorXd innovation = meas - H * x;
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    // Eigen::MatrixXd KalmanGain = P * H.transpose() * S.inverse();
    Eigen::MatrixXd KalmanGain = P * H.transpose() * S.llt().solve(Eigen::Matrix2d::Identity());

    x += KalmanGain * innovation;
    int P_size = P.cols();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(P_size, P_size);
    P = (I - KalmanGain * H) * P * (I - KalmanGain * H).transpose() + KalmanGain * R * KalmanGain.transpose();
};


void EKF::predict(){
    x = F * x;
    P = F * P * F.transpose() + Q;   
};

void EKF::update(const Eigen::VectorXd& meas, const Eigen::Vector3d& observer_pos){
    Eigen::Vector2d z_pred = nonlinear_meas_func(x, observer_pos);
    Eigen::VectorXd innovation = meas - z_pred;
    
    for(int i = 0; i < innovation.size(); i++){
        innovation(i) = normalize_angle(innovation(i));
    }
  
    // std::cout << z_pred(0) << ", " << z_pred(1) << " | ";

    H = computeJacobianH(x, observer_pos);
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    // Eigen::MatrixXd KalmanGain = P * H.transpose() * S.inverse();
    // Eigen::MatrixXd K = P * H.transpose()* S.llt().solve(Eigen::Matrix2d::Identity());
    Eigen::MatrixXd K = S.llt().solve(H * P).transpose();
    // std::cout << " innovation = " << innovation.norm() << " | ";
    x += K * innovation;
    int P_size = P.cols();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(P_size, P_size);
    P = (I - K * H) * P * (I - K * H).transpose() 
        + K * R * K.transpose();
    
};

UKF::UKF(const Eigen::VectorXd& x_, const Eigen::MatrixXd& P_, double dt_)
    :GaussianFilter(x_, P_, dt_){
    n_sig = 2 * n_x + 1;
    lambda = alpha * alpha * (n_x + kappa) - n_x;

    X_sig_pred.resize(n_x, n_sig);

    weights_m.resize(n_sig);
    weights_c.resize(n_sig);
    weights_m(0) = lambda / (n_x + lambda);
    weights_c(0) = weights_m(0) + (1 - alpha * alpha + beta);
    for (int i = 1; i < n_sig; i++) {
        weights_m(i) = weights_c(i) = 1.0 / (2.0 * (n_x + lambda));
    }
}

void UKF::setAlpha(double a_){
    alpha = a_;
    lambda = alpha * alpha * (n_x + kappa) - n_x;
    weights_m(0) = lambda / (n_x + lambda);
    weights_c(0) = weights_m(0) + (1 - alpha * alpha + beta);
    for (int i = 1; i < n_sig; i++) {
        weights_m(i) = weights_c(i) = 1.0 / (2.0 * (n_x + lambda));
    }
}

void UKF::predict(){
//    std::cout
//         << "Wc0 = " << weights_c(0)
//         << ", Wc1 = " << weights_c(1)
//         << " | ";
    Eigen::MatrixXd X_sig = Eigen::MatrixXd(n_x, n_sig);
    if(P.array().isNaN().any())
    {
        std::cerr << "P already contains NaN!" << std::endl;
    }
    Eigen::LLT<Eigen::MatrixXd> lltOfP(P);
    
    /*Comput Cholesky matrix of Covariance P for computing Sigma points later*/
    if (lltOfP.info() == Eigen::NumericalIssue) {
        // std::cerr << "!!! [UKF Warning] P is not positive definite at frame! ..." << std::endl;
        P = (P + P.transpose()).eval() * 0.5;
        P += Eigen::MatrixXd::Identity(n_x, n_x) * 1e-6;
        lltOfP.compute(P);
    }

    Eigen::MatrixXd A = lltOfP.matrixL();
    
    if (A.array().isNaN().any()) {
        std::cerr << "!!! [UKF Error] Cholesky decomposition produced NaN! ..." << std::endl;
        P = Eigen::MatrixXd::Identity(n_x, n_x) * 500.0; 
        A = P.llt().matrixL();
    }

    double shift = std::sqrt(n_x + lambda);

    /*Compute sigma points*/
    X_sig.col(0) = x;
    for (int i = 0; i < n_x; i++) {
        X_sig.col(i + 1)       = x + shift * A.col(i);
        X_sig.col(i + 1 + n_x) = x - shift * A.col(i);
    }

    /*Propagate Sigma points though motion model CV*/
    for (int i = 0; i < n_sig; i++) {
       for (int a = 0; a < 3; a++) {
            int idx = 2 * a;
            X_sig_pred(idx, i)   = X_sig(idx, i) + X_sig(idx+1, i) * dt;
            X_sig_pred(idx+1, i) = X_sig(idx+1, i);
        }
    }

    /*Recalculate mean x*/
    x.setZero();
    for (int i = 0; i < n_sig; i++) x += weights_m(i) * X_sig_pred.col(i);

    /*Recalculate covariance P*/
    P.setZero();
    for (int i = 0; i < n_sig; i++) {
        Eigen::VectorXd diff = X_sig_pred.col(i) - x;
        P += weights_c(i) * diff * diff.transpose();
    }
    P += Q;
};

void UKF::update(const Eigen::VectorXd& meas,const Eigen::Vector3d& observer_pos){
    constexpr int n_z = 2;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigP(P);
    // std::cout
    //     << "lambda_min(P)="
    //     << eigP.eigenvalues()(0)
    //     << " | ";
    //====================================================
    // Transform sigma points into measurement space
    //====================================================
    Eigen::MatrixXd Zsig(n_z, n_sig);

    for (int i = 0; i < n_sig; i++)
    {
        double px = X_sig_pred(0,i) - observer_pos(0);
        double py = X_sig_pred(2,i) - observer_pos(1);
        double pz = X_sig_pred(4,i) - observer_pos(2);

        double rho = std::sqrt(px*px + py*py);

        Zsig(0,i) = std::atan2(py,px);
        Zsig(1,i) = std::atan2(pz,rho);
    }

    // if(Zsig.array().isNaN().any())
    // {
    //     std::cerr << "Zsig contains NaN! | ";
    // }
    // if(!Zsig.allFinite())
    // {
    //     std::cerr << "Zsig contains Inf or NaN!";
    // }
    //====================================================
    // Predicted measurement mean
    //====================================================
    Eigen::Vector2d z_pred;

        //----------------------------
        // Azimuth
        //----------------------------
        double sin_sum = 0.0;
        double cos_sum = 0.0;

        for(int i = 0; i < n_sig; i++)
        {
            sin_sum += weights_m(i) * std::sin(Zsig(0,i));
            cos_sum += weights_m(i) * std::cos(Zsig(0,i));
        }

        z_pred(0) = std::atan2(sin_sum, cos_sum);

        //----------------------------
        // Elevation
        //----------------------------
        
        double ele_sum = 0.0;
        for(int i = 0; i < n_sig; i++)
            ele_sum += weights_m(i) * Zsig(1,i);
  
        z_pred(1) = ele_sum;

        // std::cout
        //     << "z_pred = ["
        //     << z_pred(0)
        //     << ", "
        //     << z_pred(1)
        //     << "]\n";
    //====================================================
    // Innovation covariance
    //====================================================
    Eigen::Matrix2d S = Eigen::Matrix2d::Zero();

    for (int i = 0; i < n_sig; i++){
        Eigen::Vector2d dz = Zsig.col(i) - z_pred;

        dz(0) = normalize_angle(dz(0));
        dz(1) = normalize_angle(dz(1));
        S += weights_c(i) * dz * dz.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigS(S);
    // std::cout
    //     << "lambda_min(S)="
    //     << eigS.eigenvalues()(0)
    //     << " | ";
    S += R;
    //====================================================
    // Cross covariance
    //====================================================
    Eigen::MatrixXd Tc = Eigen::MatrixXd::Zero(n_x, n_z);

    for (int i = 0; i < n_sig; i++){
        Eigen::VectorXd dx = X_sig_pred.col(i) - x;
        Eigen::Vector2d dz = Zsig.col(i) - z_pred;

        dz(0) = normalize_angle(dz(0));
        dz(1) = normalize_angle(dz(1));

        Tc += weights_c(i) * dx * dz.transpose();
    }

    // Kalman gain
    //====================================================
    Eigen::MatrixXd K =  Tc * S.llt().solve(Eigen::Matrix2d::Identity());
 
    // Innovation
    //====================================================
    Eigen::Vector2d innovation = meas - z_pred;
    innovation(0) = normalize_angle(innovation(0));
    // std::cout << "UKF innovation norm is " << innovation.norm() << " |\n ";
    // Update
    //====================================================
    x += K * innovation;
    P -= K * S * K.transpose();
    P = 0.5f*(P + P.transpose());
};