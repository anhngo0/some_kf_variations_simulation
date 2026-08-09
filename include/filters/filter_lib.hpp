#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <random>
#include <iostream>

const double MEAS_STD = 0.015;
const double ACC_STD = 0.5;
constexpr int STATE_DIM = 6;

double normalize_angle(double angle);
Eigen::MatrixXd computeJacobianH(const Eigen::VectorXd& state, const Eigen::Vector3d& observer_pos) ;
Eigen::VectorXd nonlinear_meas_func(const Eigen::VectorXd& mean_state, const Eigen::Vector3d& observer_pos) ;

Eigen::MatrixXd int_proc_noise_mat_Q_CV(double dt);

Eigen::Matrix2d int_meas_noise_mat_R_CV();

Eigen::MatrixXd init_Transit_Mat_CV(double dt);

struct StateCovariance {
    Eigen::VectorXd mean;
    Eigen::MatrixXd covariance;

    StateCovariance() = default;
    StateCovariance(int n){
        mean = Eigen::VectorXd::Zero(n);
        covariance = Eigen::MatrixXd::Identity(n,n) * 1e3;
    }
    StateCovariance(const Eigen::VectorXd& m, const Eigen::MatrixXd& cov){
        mean = m;
        covariance = cov;
    }
};

class GaussianFilter {
    
    protected:
        Eigen::VectorXd x; // state: [px, vx, py, vy, pz, vz]
        Eigen::MatrixXd P; /*covariance matrix*/
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
        double dt;
        void setQ(){
            Q = int_proc_noise_mat_Q_CV(dt);
        }

        void setR(){
            R  = int_meas_noise_mat_R_CV();
        }

        
    public:

        GaussianFilter(const Eigen::VectorXd& x_, const Eigen::MatrixXd& P_,double dt_){
            dt = dt_;
            x = x_;
            P = P_;
            setQ();
            setR();
        };

        virtual ~GaussianFilter() = default;
        Eigen::VectorXd getMean() const {return x;}
        Eigen::MatrixXd getCovariance() const {return P;}

        virtual void setF(const Eigen::MatrixXd& f_) = 0;
        virtual void predict() = 0;
        virtual void update(const Eigen::VectorXd& meas, const Eigen::Vector3d& observer_pos) = 0;

};