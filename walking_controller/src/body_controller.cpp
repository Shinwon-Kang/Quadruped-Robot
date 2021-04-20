#include "body_controller.h"


void BodyController::run(std::array<Eigen::Matrix4f, 4>& foot_position, const Pose &req_pose) {
    // Not Calculate ROLL, PITCH, YAW, Heigh

    // // FR
    // foot_position[0] << 1.0, 0.0, 0.0, 0.0,
    //                     0.0, 1.0, 0.0, -0.0838,
    //                     0.0, 0.0, 1.0, -0.3098,
    //                     0.0, 0.0, 0.0, 1.0;

    // // FL
    // foot_position[1] << 1.0, 0.0, 0.0, 0.0,
    //                     0.0, 1.0, 0.0, 0.0838,
    //                     0.0, 0.0, 1.0, -0.3098,
    //                     0.0, 0.0, 0.0, 1.0;
    // // RR
    // foot_position[2] << 1.0, 0.0, 0.0, 0.0,
    //                     0.0, 1.0, 0.0, -0.0838,
    //                     0.0, 0.0, 1.0, -0.3098,
    //                     0.0, 0.0, 0.0, 1.0;
    // // RL
    // foot_position[3] << 1.0, 0.0, 0.0, 0.0,
    //                     0.0, 1.0, 0.0, 0.0838,
    //                     0.0, 0.0, 1.0, -0.3098,
    //                     0.0, 0.0, 0.0, 1.0;

    for(unsigned int i = 0; i < 4; i++) {
        run(foot_position[i], *a1_base_->legs[i], req_pose);
    }
}

void BodyController::run(Eigen::Matrix4f &foot_position, QuadrupedLeg &leg, const Pose &req_pose) {
    float req_translation_x = -req_pose.position.x;
    float req_translation_y = -req_pose.position.y;

    float req_translation_z = -(leg.zero_stance().z() + req_pose.position.z); // 0.082
    float max_translation_z = -leg.zero_stance().z() * 0.25; // 0.1833

    if(req_translation_z < 0.0)
    {
        req_translation_z = 0.0;
    }
    else if(req_translation_z > max_translation_z)
    {
        req_translation_z = max_translation_z;
    }

    foot_position(0, 3) = leg.zero_stance().x();
    foot_position(1, 3) = leg.zero_stance().y();
    foot_position(2, 3) = leg.zero_stance().z();

    // Translate
    foot_position(0, 3) += req_translation_x;
    foot_position(1, 3) += req_translation_y;
    foot_position(2, 3) += req_translation_z;

    // Rotate(Z)
    foot_position(0, 3) = cosf(-req_pose.orientation.yaw) * foot_position(0, 3) - sinf(-req_pose.orientation.yaw) * foot_position(1, 3);
    foot_position(1, 3) = sinf(-req_pose.orientation.yaw) * foot_position(0, 3) + cosf(-req_pose.orientation.yaw) * foot_position(1, 3);

    // Rotate(Y)
    foot_position(0, 3) = cosf(-req_pose.orientation.pitch) * foot_position(0, 3) + sinf(-req_pose.orientation.pitch) * foot_position(2, 3);
    foot_position(2, 3) = -sinf(-req_pose.orientation.pitch) * foot_position(0, 3) + cosf(-req_pose.orientation.pitch) * foot_position(2, 3);

    // Rotate(X)
    foot_position(1, 3) = cosf(-req_pose.orientation.roll) * foot_position(1, 3) - sinf(-req_pose.orientation.roll) * foot_position(2, 3);
    foot_position(2, 3) = sinf(-req_pose.orientation.roll) * foot_position(1, 3) + cosf(-req_pose.orientation.roll) * foot_position(2, 3);

    // transformToHip
    foot_position(0, 3) -= leg.hip.x();
    foot_position(1, 3) -= leg.hip.y();
}