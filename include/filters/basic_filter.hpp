/* This file consists of baseline filter: Kalman Filter, EKF, UKF, PF*/
#pragma once
#include <Eigen/Eigenvalues>
#include "filter_lib.hpp"

class KF : public GaussianFilter{
    private:
        Eigen::MatrixXd F; /*state transition matrix*/
        Eigen::MatrixXd H; /*Measurement matrix*/
    public:
        KF(const Eigen::VectorXd& x_, const Eigen::MatrixXd& P_, double dt_) 
        : GaussianFilter(x_, P_,dt_){}
        void predict() override;
        void update(const Eigen::VectorXd& meas,const Eigen::Vector3d& observer_pos) override;

        void setF(const Eigen::MatrixXd& state_mat) override {F = state_mat;};
        void setH(const Eigen::MatrixXd& meas_mat){H=meas_mat;};
};

class EKF : public GaussianFilter{
    private:
        Eigen::MatrixXd F; /*state transition matrix*/
        Eigen::MatrixXd H; /*Measurement matrix*/
    public:
        EKF(const Eigen::VectorXd& x_, const Eigen::MatrixXd& P_,double dt_) 
        : GaussianFilter(x_, P_, dt_){}
        void predict() override;
        void update(const Eigen::VectorXd& meas,const Eigen::Vector3d& observer_pos) override;
        void setF(const Eigen::MatrixXd& state_mat) override {F = state_mat;};
        
};

class UKF : public GaussianFilter{
    private:
        int n_x = STATE_DIM;
        int n_sig;
        double alpha = 0.6; 
        double beta = 2.0;
        double kappa = 0.0;
        double lambda;
        
         // weights for Mean and Covariance
        Eigen::VectorXd weights_m;
        Eigen::VectorXd weights_c;
        Eigen::MatrixXd X_sig_pred; // Sigma points are predicted though F

    public:

        UKF(const Eigen::VectorXd& x_, const Eigen::MatrixXd& P_,double dt_);
        void setAlpha(double a_);
        void setF(const Eigen::MatrixXd& state_mat) override {};
        void predict() override;
        void update(const Eigen::VectorXd& meas,const Eigen::Vector3d& observer_pos) override;
};