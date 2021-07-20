#ifndef FOOTTRAJECTORYPLANNER_H
#define FOOTTRAJECTORYPLANNER_H

#include "Eigen/Dense"
#include "iostream"

#include "a1.h"

class FootTrajectoryPlanner {
    QuadrupedLeg *leg_; 

    Eigen::Matrix4f prev_foot_position_;

    unsigned int bz_n_control_points_;
    float bz_factorial_[13];
    float bz_ref_control_points_x_[12];
    float bz_ref_control_points_y_[12];

    float bz_control_points_x_[12];
    float bz_control_points_y_[12];

    bool has_started_;

    float length_ratio_;
    float height_ratio_;

    public:
        FootTrajectoryPlanner(QuadrupedLeg &leg):
            leg_(&leg),
            bz_n_control_points_(12),
            bz_factorial_{1.0,1.0,2.0,6.0,24.0,120.0,720.0,5040.0,40320.0,362880.0,3628800.0,39916800.0,479001600.0},
            bz_ref_control_points_x_{-0.15, -0.2805,-0.3,-0.3,-0.3, 0.0, 0.0, 0.0, 0.3032, 0.3032, 0.2826, 0.15},
            bz_ref_control_points_y_{-0.5, -0.5, -0.3611, -0.3611, -0.3611, -0.3611, -0.3611, -0.3214, -0.3214, -0.3214, -0.5, -0.5},
            has_started_(false),
            length_ratio_(0.0),
            height_ratio_(0.0)
        {
        }

        void run(Eigen::Matrix4f &foot_position, float step_length, float rotation,
                 float swing_phase_signal, float stance_phase_signal);
        void updateControlPointsLength(float step_length);
        void updateControlPointsHeight(float swing_height);
};

#endif