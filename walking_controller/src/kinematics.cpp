#include "walking_controller/kinematics.h"

void Kinematics::inverse(float (&joint_positions)[12], std::array<Eigen::Matrix4f, 4>& foot_positions, Eigen::VectorXd pose) {
    float calculated_joints[12];

    for(unsigned int i = 0; i < 4; i++) {
        inverse(calculated_joints[(i*3)], calculated_joints[(i*3) + 1], calculated_joints[(i*3) + 2], 
                *a1_base_->legs[i], foot_positions[i], pose(0), pose(1), pose(2));
                    
        //check if any leg has invalid calculation, if so disregard the whole plan
        if(std::isnan(calculated_joints[(i*3)]) || std::isnan(calculated_joints[(i*3) + 1]) || std::isnan(calculated_joints[(i*3) + 2]))
        {
            return;
        }
    }

    for(unsigned int i = 0; i < 12; i++) {
        joint_positions[i] = calculated_joints[i];
    }
}

void Kinematics::inverse(float &hip_joint, float &upper_leg_joint, float &lower_leg_joint, 
             QuadrupedLeg &leg, Eigen::Matrix4f& foot_position,
             double pose_x, double pose_y, double pose_z) {

    Eigen::Vector3f temp_foot_pos;
    temp_foot_pos.x() = foot_position(0, 3);
    temp_foot_pos.y() = foot_position(1, 3);
    temp_foot_pos.z() = foot_position(2, 3);

    float l0 = 0.0f;

    for(unsigned int i = 1; i < 4; i++) {
        l0 += leg.joints[i]->y();
        // l0 += leg.joints[i]->y() + pose_y;
    }

    float l1 = -sqrtf(pow(leg.lower_leg.x(), 2) + pow(leg.lower_leg.z(), 2));
    // float l1 = -sqrtf(pow((leg.lower_leg.x() + pose_x), 2) + pow((leg.lower_leg.z()), 2));

    float ik_alpha = acosf(leg.lower_leg.x() / l1) - (M_PI / 2);

    float l2 = - sqrtf(pow(leg.foot.x(), 2) + pow(leg.foot.z(), 2));
    // float l2 = - sqrtf(pow((leg.foot.x() + pose_x), 2) + pow((leg.foot.z()), 2));

    float ik_beta = acosf(leg.foot.x() / l2) - (M_PI / 2);

    float x = temp_foot_pos.x();
    float y = temp_foot_pos.y();
    float z = temp_foot_pos.z();

    hip_joint = -(atanf(y / z) - ((M_PI/2) - acosf(-l0 / sqrtf(pow(y, 2) + pow(z, 2)))));

    // Rotate(X)
    temp_foot_pos.y() = cosf(-hip_joint) * temp_foot_pos.y() - sinf(-hip_joint) * temp_foot_pos.z();
    temp_foot_pos.z() = sinf(-hip_joint) * temp_foot_pos.y() + cosf(-hip_joint) * temp_foot_pos.z();

    // Translate
    temp_foot_pos.x() += -leg.upper_leg.x() - pose_x;
    temp_foot_pos.y() += 0.0f - pose_y;
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
            upper_leg_joint = upper_leg_joint + M_PI;
        }
    } else {
        if(upper_leg_joint > 0)
        {
            upper_leg_joint = upper_leg_joint + M_PI;
        }
    }
}

Eigen::Matrix<float, 3, 4> Kinematics::getLocalPositions(Eigen::Matrix<float, 3, 4> leg_positions, 
                                             float dx, float dy, float dz, 
                                             float r, float p, float y) {

    Eigen::Matrix4f leg_positions_m;
    leg_positions_m.block<3, 4>(0, 0) = leg_positions;
    for(int i = 0; i < 4; i++) {
        leg_positions_m(3, i) = 1.0;
    }

    Eigen::Matrix4f T_blwbl = homog_transform(dx, dy, dz, r, p, y);

    // Transformation matrix, base_link_world => FR1
    Eigen::Matrix4f T_blwFR1 = T_blwbl * homog_transform(
        0.5 * body_length, -0.5 * body_width, 0.0,
        PI/2, -PI/2, 0.0);
    
    // Transformation matrix, base_link_world => FL1
    Eigen::Matrix4f T_blwFL1 = T_blwbl * homog_transform(
        0.5 * body_length, 0.5 * body_width, 0.0,
        PI/2, -PI/2, 0.0);

    // Transformation matrix, base_link_world => RR1
    Eigen::Matrix4f T_blwRR1 = T_blwbl * homog_transform(
        -0.5 * body_length, -0.5 * body_width, 0.0,
        PI/2, -PI/2, 0.0);
    
    // Transformation matrix, base_link_world => RL1
    Eigen::Matrix4f T_blwRL1 = T_blwbl * homog_transform(
        -0.5 * body_length, 0.5 * body_width, 0.0,
        PI/2, -PI/2, 0.0);

    // apply transformations
    // FR
    leg_positions_m.col(0) = homog_transform_inverse(T_blwFR1) *
                leg_positions_m.col(0);

    // FL
    leg_positions_m.col(1) = homog_transform_inverse(T_blwFL1) *
                leg_positions_m.col(1);
    
    // RR
    leg_positions_m.col(2) = homog_transform_inverse(T_blwRR1) *
                leg_positions_m.col(2);

    // RL
    leg_positions_m.col(3) = homog_transform_inverse(T_blwRL1) *
                leg_positions_m.col(3);

    return leg_positions_m.block<3,4>(0,0); 
}

std::vector<double> Kinematics::wholeBodyIK(Eigen::Matrix<float, 3, 4> leg_positions, 
                                            float dx, float dy, float dz, 
                                            float r, float p, float y) {

    Eigen::Matrix<float, 3, 4> positions = getLocalPositions(leg_positions, dx, dy, dz, r, p, y);
    std::vector<double> angles;

    for(int i = 0; i < 4; i++) {
        double x = positions.col(i)[0];
        double y = positions.col(i)[1];
        double z = positions.col(i)[2];

        double F = sqrt(x*x + y*y - hip*hip);
        double G = F - a1;
        double H = sqrt(G*G + z*z);

        double theta1 = atan2(y, x) + atan2(F, hip * pow((-1), i));

        double D = (H*H - thigh*thigh - calf*calf) / (2*thigh*calf);

        double theta4 = -atan2((sqrt(1-D*D)), D);

        double theta3 = atan2(z, G) - atan2(calf * sin(theta4), thigh + calf * cos(theta4));

        // angle : theta1, theta3, theta4
        angles.push_back(-theta1);
        angles.push_back(theta3);
        angles.push_back(theta4);
    }

    return angles;
}

void Kinematics::transformToHip(Eigen::Matrix4f& foot_position, const QuadrupedLeg &leg) {
    foot_position(0, 3) -= leg.hip.x();
    foot_position(1, 3) -= leg.hip.y();
}

void Kinematics::transformToBase(Eigen::Matrix4f& foot_position, const QuadrupedLeg &leg) {
    foot_position(0, 3) += leg.hip.x();
    foot_position(1, 3) += leg.hip.y();
    foot_position(2, 3) += leg.hip.z();
}