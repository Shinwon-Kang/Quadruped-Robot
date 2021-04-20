#include "leg_controller.h"

float LegController::min_max_velocity(float v, float min_v, float max_v) {
    return ((v)<(min_v)?(min_v):((v)>(max_v)?(max_v):(v)));
}

float LegController::raibertHeuristic(float st_duration, float v) {
    return (st_duration / 2.0f) * v;
}

void LegController::transfromLegToNextStep(float &step_length, float &rotation, QuadrupedLeg &leg,
                            float step_x, float step_y, float theta) {

    Eigen::Vector3f t = leg.zero_stance();

    // Translate
    t.x() += step_x;
    t.y() += step_y;
    t.z() += 0.0f;

    // Rotate(Z)
    t.x() = cosf(theta) * t.x() - sinf(theta) * t.y();
    t.y() = sinf(theta) * t.x() + cosf(theta) * t.y();


    float delta_x = t(0) - leg.zero_stance().x();
    float delta_y = t(1) - leg.zero_stance().y();

    step_length = sqrtf(pow(delta_x, 2) + pow(delta_y, 2)) * 2.0f;

    rotation = atan2f(delta_y, delta_x);
}

void LegController::run(std::array<Eigen::Matrix4f, 4>& foot_positions, Velocities& req_vel, PhaseSignalGenerator::Time time = PhaseSignalGenerator::now()) {
    // Max Linear Velocity X : 0.5
    // Max Linear Velocity Y : 0.25
    // Max Angular Velocity Z : 1.0

    req_vel.linear.x = min_max_velocity(req_vel.linear.x, -linear_velocity_x_, linear_velocity_x_);
    req_vel.linear.y = min_max_velocity(req_vel.linear.y, -linear_velocity_y_, linear_velocity_y_);
    req_vel.angular.z = min_max_velocity(req_vel.angular.z, -angular_velocity_z_, angular_velocity_z_);

    float tangential_velocity = req_vel.angular.z * a1_base_->FR_.center_to_nominal();
    float velocity =  sqrtf(pow(req_vel.linear.x, 2) + pow(req_vel.linear.y + tangential_velocity, 2));

    float step_x = raibertHeuristic(stance_duration_, req_vel.linear.x);
    float step_y = raibertHeuristic(stance_duration_, req_vel.linear.y);
    float step_theta = raibertHeuristic(stance_duration_, tangential_velocity);

    float theta = sinf((step_theta / 2) / a1_base_->FR_.center_to_nominal()) * 2;

    float step_lengths[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float trajectory_rotations[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float sum_of_steps = 0.0f;

    for(int i = 0; i < 4; i++) {
        transfromLegToNextStep(step_lengths[i], trajectory_rotations[i], *a1_base_->legs[i], step_x, step_y, theta);
        sum_of_steps += step_lengths[i];
    }

    // 1. Phase_generator
    phase_signal_generator_.run(velocity, sum_of_steps / 4.0f, time);

    // 2. Trajectory_planners
    for(unsigned int i = 0; i < 4; i++) {
        foot_trajectory_planner_[i]->run(foot_positions[i], step_lengths[i], trajectory_rotations[i], 
                                        phase_signal_generator_.swing_phase_signal[i], phase_signal_generator_.stance_phase_signal[i]);
    }
}