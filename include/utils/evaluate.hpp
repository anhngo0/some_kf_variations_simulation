#pragma once
#include <Eigen/Dense>
#include <vector>

struct RMSE_per_Frame{
    double pos;
    double vel;
    double frame;
    RMSE_per_Frame(double rmse_pos, double rmse_vel, double fr): pos(rmse_pos), vel(rmse_vel), frame(fr){}
    RMSE_per_Frame(){}
};

RMSE_per_Frame rmse_per_frame(
    const Eigen::VectorXd& est, 
    const Eigen::VectorXd& gt, 
    double frame
);