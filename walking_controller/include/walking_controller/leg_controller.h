#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H

#include "Eigen/Dense"
#include "array"
#include "iostream"
#include "math.h"

#include "a1.h"
#include "a1_components.h"
#include "phase_signal_generator.h"
#include "foot_trajectory_planner.h"

class LegController {
    A1 *a1_base_;
    // PhaseSignalGenerator phase_signal_generator_;
    FootTrajectoryPlanner *foot_trajectory_planner_[4];

    //TODO: 파라미터로 만들어야됨 하날로 관리하게끔
    float stance_duration_ = 0.25;

    float linear_velocity_x_ = 0.5;
    float linear_velocity_y_ = 0.3;

    float angular_velocity_z_ = 1.0;

    public:
        LegController(A1 &a1_base, PhaseSignalGenerator::Time time = PhaseSignalGenerator::now()):
            a1_base_(&a1_base),
            phase_signal_generator_(a1_base, time),
            FL(a1_base_->FL_),
            FR(a1_base_->FR_),
            RL(a1_base_->RL_),
            RR(a1_base_->RR_)
        {
            foot_trajectory_planner_[0] = &FL;
            foot_trajectory_planner_[1] = &FR;
            foot_trajectory_planner_[2] = &RL;
            foot_trajectory_planner_[3] = &RR;
        }

        float min_max_velocity(float v, float min_v, float max_v);
        float raibertHeuristic(float st_duration, float v);
        void transfromLegToNextStep(float &step_length, float &rotation, QuadrupedLeg &leg,
                                    float step_x, float step_y, float theta);
        void run(std::array<Eigen::Matrix4f, 4>& foot_position, Velocities& req_velh,
                 PhaseSignalGenerator::Time time);

        FootTrajectoryPlanner FL;
        FootTrajectoryPlanner FR;
        FootTrajectoryPlanner RL;
        FootTrajectoryPlanner RR;

        PhaseSignalGenerator phase_signal_generator_;
};

#endif