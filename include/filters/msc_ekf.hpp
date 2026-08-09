#include "filter_lib.hpp"

/*========================================================================================*/

/*Obtained by a (3,2) Euler sequence with euler angles (phi = pi/2 - beta; delta = pi / 2 - epsilon)
~ rotate Z axis from x yo y an angle phi = pi/2 - beta 
  then rotate Y axis from z to x an angle delta = pi/2  - epsilon*/
Eigen::Matrix3d rotate_mat_from_Tframe_to_Sframe(double beta, double epsilon) {
    return Eigen::Matrix3d(
        sin(epsilon) * sin(beta), sin(epsilon) * cos(beta), -cos(epsilon),
        - cos(beta)             , sin(beta)               , 0,
        cos(epsilon) * sin(beta), cos(epsilon) * cos(beta), sin(epsilon)      
    );
}

/*========================================================================================*/

/*
    r : range value (scalar)
    beta: bearing angle
    epsilon : elevation angle
    zeta = ln(1/r) 

    ---------------------------
    dot_x : derivation of x (eg: dot_beta = bearing velocity)
*/

struct msc_state{
    double w; /* = dot_beta * cos(epsilon)*/
    double dot_epsilon; 
    double dot_zeta; /* =  dot_r / r */
    double beta;
    double epsilon;
    double reverse_range; /* = 1/r*/
    msc_state(){}
    msc_state(double w_value, double dot_ep_value, double dot_zeta_value, double beta_value, double ep_value, double reverse_range_value){
        w = w_value; dot_epsilon = dot_ep_value;
        dot_zeta = dot_zeta_value; beta = beta_value;
        epsilon = ep_value; reverse_range = reverse_range_value;
    }
};

/*CV cartesian state x = {x, y, z, vx, vy, vz}*/
msc_state convert_CVCartesian_to_MSC (Eigen::VectorXd cartesian_state){
    double x = cartesian_state(0); double vx = cartesian_state(1);
    double y = cartesian_state(2); double vy = cartesian_state(3);
    double z = cartesian_state(4); double vz = cartesian_state(5);
    
    double range = std::sqrt(x * x + y * y + z * z);
    double rev_range = 1.0 / range;
    
    /*-------*/
    double eps = atan2(z , std::sqrt(x*x + y*y));
    double beta = atan2(y , x);
    
    double dot_zeta = (x * vx + y * vy + z * vz) / (range * range);
    double dot_eps = ((x*x + y*y)*vz - z * (vx*x + vy*y)) / (range * range * std::sqrt(x*x + y*y));
    double dot_beta = (x * vy - y*vx) / (x*x+y*y);
    double w = dot_beta * cos(eps);

    return msc_state(
        w, 
        dot_eps,
        dot_zeta,
        beta,
        eps,
        rev_range
    );
}



