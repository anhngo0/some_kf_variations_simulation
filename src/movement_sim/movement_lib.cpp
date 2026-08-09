#include "../../include/movement_sim/movement_lib.hpp"

/*2D : x, y*/
GT_State_Per_Frame CT::move(double frame) const {
    if (!contains(frame)) {
        std::string error_mes =
            "Can not access frame " + std::to_string(frame) +
            " in CT segment (" +
            std::to_string(start_frame) + ", " +
            std::to_string(end_frame) + ")";

        throw std::runtime_error(error_mes);
    }

    double dt = frame - start_frame;

    double heading = std::atan2(start_point.vel.y(), start_point.vel.x());; /*motion vector, rotate in Oxy*/
    double speed = start_point.vel.head<2>().norm(); /*head<2>() take the first 2 values x and y then norm ~ std::sqrt(x*x + y*y)*/
    double theta = heading + omega * dt;

    Eigen::Vector3d pos = start_point.pos;
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();

    if (std::abs(omega) < 1e-6) {
        vel << speed * std::cos(heading), speed * std::sin(heading), 0.0;
        pos += vel * dt;

    } else {

        double r = speed / omega;
        pos.x() += r * (std::sin(theta) - std::sin(heading));
        pos.y() -= r * (std::cos(theta) - std::cos(heading));
        
        vel << speed * std::cos(theta), speed * std::sin(theta), 0.0;

        acc << -speed * omega * std::sin(theta),
                speed * omega * std::cos(theta),
                0.0;
    }

    return GT_State_Per_Frame(pos, vel, acc, frame);
}

GT_State_Per_Frame CA::move(double frame) const {
    if(!contains(frame)){
        std::string error_mes = "Can not access frame " +  std::to_string(frame) 
                                + " in CA segment start at stack frame (" 
                                + std::to_string(start_frame) + " x " + std::to_string(end_frame) + " )\n";
        throw std::runtime_error(error_mes);
    }
    double dt = frame - start_frame;
    Eigen::Vector3d pos = start_point.pos + dt*start_point.vel + 0.5 * dt * dt * start_point.acc;
    Eigen::Vector3d vel = start_point.vel + dt * start_point.acc;
    Eigen::Vector3d acc = start_point.acc;

    return GT_State_Per_Frame(pos,vel,acc, frame);
}

GT_State_Per_Frame CV::move(double frame) const {
    if(!contains(frame)){
        std::string error_mes = "Can not access frame " +  std::to_string(frame) 
                                + " in CV segment start at stack frame (" 
                                + std::to_string(start_frame) + " x " + std::to_string(end_frame) + " )\n";
        throw std::runtime_error(error_mes);
    }
    double dt = frame - start_frame;
    Eigen::Vector3d pos = start_point.pos + dt*start_point.vel;
    Eigen::Vector3d vel = start_point.vel;
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    return GT_State_Per_Frame(pos,vel,acc, frame);
}

