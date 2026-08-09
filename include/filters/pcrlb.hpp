#include "filter_lib.hpp"

class PCRLB {
    Eigen::MatrixXd Q_inv;
    Eigen::Matrix2d R_inv;
    Eigen::MatrixXd F;
    Eigen::MatrixXd I; /*Fisher Information Matrix*/
    double dt;

    public:
        PCRLB(const Eigen::MatrixXd& P, double dt);
        void setF(const Eigen::MatrixXd& F_){F = F_;};
        Eigen::Vector2d convertIToRmseVector() const; //(rmse position, rmse velocity)

        void update(const Eigen::VectorXd& x_true, const Eigen::Vector3d& observer_pos);
};