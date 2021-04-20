#include "kinematics.h"

void Kinematics::inverse(float (&joint_positions)[12], std::array<Eigen::Matrix4f, 4>& foot_positions) {
    float calculated_joints[12];

    for(unsigned int i = 0; i < 4; i++) {
        inverse(calculated_joints[(i*3)], calculated_joints[(i*3) + 1], calculated_joints[(i*3) + 2], 
                *a1_base_->legs[i], foot_positions[i]);
                    
        //check if any leg has invalid calculation, if so disregard the whole plan
        if(isnan(calculated_joints[(i*3)]) || isnan(calculated_joints[(i*3) + 1]) || isnan(calculated_joints[(i*3) + 2]))
        {
            return;
        }
    }

    for(unsigned int i = 0; i < 12; i++) {
        joint_positions[i] = calculated_joints[i];
    }
}

void Kinematics::inverse(float &hip_joint, float &upper_leg_joint, float &lower_leg_joint, 
             QuadrupedLeg &leg, Eigen::Matrix4f& foot_position) {

    Eigen::Vector3f temp_foot_pos;
    temp_foot_pos.x() = foot_position(0, 3);
    temp_foot_pos.y() = foot_position(1, 3);
    temp_foot_pos.z() = foot_position(2, 3);

    float l0 = 0.0f;

    for(unsigned int i = 1; i < 4; i++) {
        l0 += leg.joints[i]->y();
    }

    float l1 = -sqrtf(pow(leg.lower_leg.x(), 2) + pow(leg.lower_leg.z(), 2));
    float ik_alpha = acosf(leg.lower_leg.x() / l1) - (M_PI / 2);

    float l2 = - sqrtf(pow(leg.foot.x(), 2) + pow(leg.foot.z(), 2));
    float ik_beta = acosf(leg.foot.x() / l2) - (M_PI / 2);

    float x = temp_foot_pos.x();
    float y = temp_foot_pos.y();
    float z = temp_foot_pos.z();

    hip_joint = -(atanf(y / z) - ((M_PI/2) - acosf(-l0 / sqrtf(pow(y, 2) + pow(z, 2)))));

    // Rotate(X)
    temp_foot_pos.y() = cosf(-hip_joint) * temp_foot_pos.y() - sinf(-hip_joint) * temp_foot_pos.z();
    temp_foot_pos.z() = sinf(-hip_joint) * temp_foot_pos.y() + cosf(-hip_joint) * temp_foot_pos.z();

    // Translate
    temp_foot_pos.x() += -leg.upper_leg.x();
    temp_foot_pos.y() += 0.0f;
    temp_foot_pos.z() += -leg.upper_leg.z();

    x = temp_foot_pos.x();
    y = temp_foot_pos.y();
    z = temp_foot_pos.z();

    float target_to_foot = sqrtf(pow(x, 2) + pow(z, 2));
    if(target_to_foot >= (abs(l1) + abs(l2))) {
        return;
    }

    lower_leg_joint = leg.knee_direction() * acosf((pow(z, 2) + pow(x, 2) - pow(l1 ,2) - pow(l2 ,2)) / (2 * l1 * l2));
    upper_leg_joint = (atanf(x / z) - atanf((l2 * sinf(lower_leg_joint)) / (l1 + (l2 * cosf(lower_leg_joint)))));
    lower_leg_joint += ik_beta - ik_alpha;
    upper_leg_joint += ik_alpha;

    if(leg.knee_direction() < 0) {
        if(upper_leg_joint < 0)
        {
            upper_leg_joint = upper_leg_joint +  M_PI;
        }
    } else {
        if(upper_leg_joint > 0)
        {
            upper_leg_joint = upper_leg_joint +  M_PI;
        }
    }
}