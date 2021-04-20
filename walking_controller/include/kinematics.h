#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "Eigen/Dense"
#include "array"
#include "math.h"

#include "a1.h"

class Kinematics {
    A1 *a1_base_;

    public:
        Kinematics(A1 &a1_base):
            a1_base_(&a1_base)
        {
        }

        void inverse(float (&joint_positions)[12], std::array<Eigen::Matrix4f, 4>& foot_positions);
        void inverse(float &hip_joint, float &upper_leg_joint, float &lower_leg_joint, 
                     QuadrupedLeg &leg, Eigen::Matrix4f& foot_position);
};
#endif