#include "foot_trajectory_planner.h"

void FootTrajectoryPlanner::updateControlPointsLength(float step_length) {
    float new_length_ratio = step_length / 0.4f;

    if(length_ratio_ != new_length_ratio) {
        length_ratio_ = new_length_ratio;
        for(unsigned int i = 0; i < 12; i++) {
            if(i == 0) {
                bz_control_points_x_[i] = -step_length / 2.0f;
            } else if(i == 11) {
                bz_control_points_x_[i] = step_length / 2.0f;
            } else {
                bz_control_points_x_[i] = bz_ref_control_points_x_[i] * length_ratio_;
            }
        }
    }
}

void FootTrajectoryPlanner::updateControlPointsHeight(float swing_height) {
    float new_height_ratio = swing_height / 0.15f;

    if(height_ratio_ != new_height_ratio) {
        height_ratio_ = new_height_ratio;

        for(unsigned int i = 0; i < 12; i++) {
            bz_control_points_y_[i] = -((bz_ref_control_points_y_[i] * height_ratio_) + (0.5f * height_ratio_));
        }
    }
}

void FootTrajectoryPlanner::run(Eigen::Matrix4f &foot_position, float step_length, float rotation,
                                float swing_phase_signal, float stance_phase_signal) {
    // TODO: parameter                                     
    updateControlPointsHeight(0.07);

    if(!has_started_) {
        has_started_ = true;
        prev_foot_position_ = foot_position;
    }

    if(step_length == 0.0f) {
        prev_foot_position_ = foot_position;
        leg_->gait_phase(1);
        
        return;
    }

    updateControlPointsLength(step_length);

    int n = bz_n_control_points_ - 1;
    float x = 0.0f;
    float y = 0.0f;

    if (stance_phase_signal > swing_phase_signal) {
        // Stance Mode update
        leg_->gait_phase(1);
        
        x = (step_length / 2) * (1 - (2 * stance_phase_signal));
        y = -0.0 * cosf((M_PI * x) / step_length);
    } else if (stance_phase_signal < swing_phase_signal) {
        // Swing Mode update
        leg_->gait_phase(0);

        for(unsigned int i = 0; i < bz_n_control_points_; i++) {
            float coeff = bz_factorial_[n] / (bz_factorial_[i] * bz_factorial_[n - i]);

            x += coeff * pow(swing_phase_signal, i) * pow((1 - swing_phase_signal), (n - i)) * bz_control_points_x_[i];
            y -= coeff * pow(swing_phase_signal, i) * pow((1 - swing_phase_signal), (n - i)) * bz_control_points_y_[i];
        }
    }

    foot_position(0, 3) += x * cosf(rotation);
    foot_position(1, 3) += x * sinf(rotation);
    foot_position(2, 3) += y;


    if((swing_phase_signal == 0.0f && stance_phase_signal == 0.0f) && step_length > 0.0f)
    {
        foot_position = prev_foot_position_;
    }

    prev_foot_position_ = foot_position;
}