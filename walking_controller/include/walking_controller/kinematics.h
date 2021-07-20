#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "Eigen/Dense"
#include "array"
#include "math.h"
#include "vector"
#include "iostream"

#include "a1.h"
#include "util_transformations.h"

class Kinematics {
    A1 *a1_base_;

    // 원래는 0.1805 -> 0.183 사용
    // float body_length = 0.366;
    float body_length = 0.361;
    float body_width = 0.094;    

    float a1 = 0.0;
    // 0.0838 인데?
    // float hip = 0.08505;
    float hip = 0.0838;
    float thigh = 0.2;
    float calf = 0.2;

    public:
        Kinematics(A1 &a1_base):
            a1_base_(&a1_base)
        {
        }

        void inverse(float (&joint_positions)[12], std::array<Eigen::Matrix4f, 4>& foot_positions, Eigen::VectorXd pose);
        void inverse(float &hip_joint, float &upper_leg_joint, float &lower_leg_joint, 
                     QuadrupedLeg &leg, Eigen::Matrix4f& foot_position,
                     double pose_x, double pose_y, double pose_z);



        std::vector<double> wholeBodyIK(Eigen::Matrix<float, 3, 4> leg_positions, 
                                        float dx, float dy, float dz, 
                                        float r, float p, float y);
        Eigen::Matrix<float, 3, 4> getLocalPositions(Eigen::Matrix<float, 3, 4> leg_positions, 
                                                     float dx, float dy, float dz, 
                                                     float r, float p, float y);



        void transformToHip(Eigen::Matrix4f& foot_position, const QuadrupedLeg &leg);    
        void transformToBase(Eigen::Matrix4f& foot_position, const QuadrupedLeg &leg);  
};
#endif