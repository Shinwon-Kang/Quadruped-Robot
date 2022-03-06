#ifndef BODYCONTROLLER_H
#define BODYCONTROLLER_H

#include "array"
#include "Eigen/Dense"
#include <iostream>
#include "math.h"

#include "a1_components.h"
#include "a1.h"

class BodyController {
    A1 *a1_base_;

    public:
        BodyController(A1 &a1_base):
            a1_base_(&a1_base)
        {
        }

        void run(std::array<Eigen::Matrix4f, 4>& foot_position, const Pose &req_pose);
        void run(Eigen::Matrix4f &foot_position, QuadrupedLeg &leg, const Pose &req_pose);
};

#endif