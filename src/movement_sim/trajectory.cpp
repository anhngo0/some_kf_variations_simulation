#include "../../include/movement_sim/trajectory.hpp"

void Trajectory::addSegment(std::unique_ptr<MotionSegment> seg){
    segments.emplace_back(std::move(seg));
}

void Trajectory::generate(double frame_step){
    traj.clear();
    GT_State_Per_Frame last_state_per_segment;
    for(auto& seg: segments){
        /*check if this is not the first segment of trajectory, then assign
        state of last segment to this first state of this segment*/
        if(!seg->isStartPointExist())
            seg->setStartPoint(last_state_per_segment);

        for (double t = seg->getStartFrame();
            t <= seg->getEndFrame();
            t += frame_step)
        {
            if(t == seg->getStartFrame())
                continue; /*avoid repeating of the last state (prev segment) and the first state(this segment)*/
            traj.push_back(seg->move(t));
        }
        last_state_per_segment = traj.back();
    }
}