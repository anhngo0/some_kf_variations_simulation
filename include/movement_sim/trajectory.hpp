#pragma once
#include "movement_lib.hpp"
#include <memory>

class Trajectory{
    private:
        std::vector<GT_State_Per_Frame> traj;
        std::vector<std::unique_ptr<MotionSegment>> segments;

        public:
            Trajectory() = default;
            ~Trajectory() = default;

            void addSegment(std::unique_ptr<MotionSegment> seg);

            void generate(double frame_step);

            const std::vector<GT_State_Per_Frame>& getGroundTruth() const {
                return traj;
            }

            const GT_State_Per_Frame getLastState(){return traj.back();}
};