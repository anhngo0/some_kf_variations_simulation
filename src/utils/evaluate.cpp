#include "../../include/utils/evaluate.hpp"
RMSE_per_Frame rmse_per_frame(
    const Eigen::VectorXd& est, 
    const Eigen::VectorXd& gt, 
    double frame
){
    double mse_pos = (est(0) - gt(0)) * (est(0) - gt(0)) + (est(4) - gt(4)) * (est(4) - gt(4)) + (est(2) - gt(2)) * (est(2) - gt(2));
    double rmse_pos = std::sqrt(mse_pos / 3.0);

    double mse_vel = (est(3) - gt(3)) * (est(3) - gt(3)) + (est(1) - gt(1)) * (est(1) - gt(1)) + (est(5) - gt(5)) * (est(5) - gt(5));
    double rmse_vel = std::sqrt(mse_vel / 3.0);

    return RMSE_per_Frame(rmse_pos, rmse_vel, frame);
}