#pragma once
#include <cmath>
#include <vector>
#include <Eigen/Dense>

struct GT_State_Per_Frame {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero(); 
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    double frame = -1.0;
    GT_State_Per_Frame(
        const Eigen::Vector3d& p,
        const Eigen::Vector3d& v,
        const Eigen::Vector3d& a,
        double fr 
    ): pos(p), vel(v), acc(a), frame(fr){}
    GT_State_Per_Frame(){}
    Eigen::VectorXd toCVStateVector(){
        Eigen::VectorXd res(6);
        res << pos(0), vel(0), pos(1),vel(1), pos(2), vel(2);
        return res;
    }
    Eigen::Vector3d getPos(){return pos;}
    Eigen::Vector3d getAcc(){return acc;}

};

class MotionSegment {

    protected:
        double start_frame = 0.0;
        double end_frame = 0.0;
        GT_State_Per_Frame start_point;
    
    public:
        MotionSegment(double s, double e, const GT_State_Per_Frame& p): start_frame(s), end_frame(e), start_point(p){}
        MotionSegment(double s, double e): start_frame(s), end_frame(e){}
        
        virtual ~MotionSegment() = default;
        virtual GT_State_Per_Frame move(double frame) const = 0;

        bool contains(double frame) const {
            return frame >= start_frame && frame <= end_frame;
        }
        /*if this segment is not the fist traj, then set startPoint equal last state of previos segment*/
        void setStartPoint(const GT_State_Per_Frame& start){start_point = start;} 
        bool isStartPointExist() const {return start_point.frame != -1.0;} /*check if this segment is the first segment in trajectory*/
        double getStartFrame() const {return start_frame;}
        double getEndFrame() const {return end_frame;}
};

class CT : public MotionSegment{
    private: 
        double omega; /*w (rad/s)*/
    public:
        CT(
            double s, double e, const GT_State_Per_Frame& p, double w_
        ) : MotionSegment(s, e, p), omega(w_) {}
        CT(
            double s, double e, double w_
        ) : MotionSegment(s, e), omega(w_) {}
        GT_State_Per_Frame move(double frame) const override;
};

class CV : public MotionSegment{
 
    public:
        CV(double s, double e) : MotionSegment(s, e){}
        CV(double s, double e, const GT_State_Per_Frame& p) : MotionSegment(s, e, p){}
        
        GT_State_Per_Frame move(double frame) const override;
};

class CA : public MotionSegment{

    public:
        CA(double s, double e) : MotionSegment(s, e) {}
        CA(double s, double e, const GT_State_Per_Frame& p) : MotionSegment(s, e, p) {}
        GT_State_Per_Frame move(double frame) const override;
};