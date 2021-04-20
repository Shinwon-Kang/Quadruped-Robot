#ifndef A1_H
#define A1_H

#include "math.h"
#include "Eigen/Dense"

#include "a1_components.h"

class Joint {
    float theta_;
    
    Point translation_;
    Euler rotation_;

    public:
        Joint() {
            theta_ = 0.0;
        }

        Joint(float pos_x, float pos_y, float pos_z,
              float or_r,  float or_p,  float or_y) {
            translation_.x = pos_x;
            translation_.y = pos_y;
            translation_.z = pos_z;
            rotation_.roll = or_r;
            rotation_.pitch = or_p;
            rotation_.yaw = or_y;
        }

        void setOrigin(float x, float y, float z,
                       float roll, float pitch, float yaw) {
            translation_.x = x;
            translation_.y = y;
            translation_.z = z;
            rotation_.roll = roll;
            rotation_.pitch = pitch;
            rotation_.yaw = yaw;

        }
        
        const float x() const {
            return translation_.x;
        }

        const float y() const {
            return translation_.y;
        }

        const float z() const {
            return translation_.z;
        }

};

class QuadrupedLeg {
    bool gait_phase_;
    int knee_direction_;

    public:
        Joint *joints[4];

        Joint hip;
        Joint upper_leg;
        Joint lower_leg;
        Joint foot;

        QuadrupedLeg():
            gait_phase_(1),
            knee_direction_(-1)
        {
            joints[0] = &hip;
            joints[1] = &upper_leg;
            joints[2] = &lower_leg;
            joints[3] = &foot;        
        }

        float center_to_nominal() {
            float x = hip.x() + upper_leg.x();
            float y = hip.y() + upper_leg.y();

            return sqrtf(pow(x,2) + pow(y,2));
        }

        Eigen::Vector3f zero_stance() {
            Eigen::Vector3f t;
            // TODO: com_x_translation
            t.x() = hip.x() + upper_leg.x() - 0.025;

            t.y() = hip.y() + upper_leg.y();

            t.z() = hip.z() + upper_leg.z() + lower_leg.z() + foot.z();

            return t;
        }

        bool gait_phase(bool phase) {
            gait_phase_ = phase;
        }

        int knee_direction() {
            return knee_direction_;
        }
};

class A1 {
    public:
        QuadrupedLeg *legs[4];

        QuadrupedLeg FR_;
        QuadrupedLeg FL_;
        QuadrupedLeg RR_;
        QuadrupedLeg RL_;

        A1() {
            FR_.hip.setOrigin(0.1805, -0.047, 0.0, 0.0, 0.0, 0.0);
            FR_.upper_leg.setOrigin(0.0, -0.0838, 0.0, 0.0, 0.0, 0.0);
            FR_.lower_leg.setOrigin(0.0, 0.0, -0.2, 0.0, 0.0, 0.0);
            FR_.foot.setOrigin(0.0, 0.0, -0.2, 0.0, 0.0, 0.0);

            FL_.hip.setOrigin(0.1805, 0.047, 0.0, 0.0, 0.0, 0.0);
            FL_.upper_leg.setOrigin(0.0, 0.0838, 0.0, 0.0, 0.0, 0.0);
            FL_.lower_leg.setOrigin(0.0, 0.0, -0.2, 0.0, 0.0, 0.0);
            FL_.foot.setOrigin(0.0, 0.0, -0.2, 0.0, 0.0, 0.0);

            RR_.hip.setOrigin(-0.1805, -0.047, 0.0, 0.0, 0.0, 0.0);
            RR_.upper_leg.setOrigin(0.0, -0.0838, 0.0, 0.0, 0.0, 0.0);
            RR_.lower_leg.setOrigin(0.0, 0.0, -0.2, 0.0, 0.0, 0.0);
            RR_.foot.setOrigin(0.0, 0.0, -0.2, 0.0, 0.0, 0.0);

            RL_.hip.setOrigin(-0.1805, 0.047, 0.0, 0.0, 0.0, 0.0);
            RL_.upper_leg.setOrigin(0.0, 0.0838, 0.0, 0.0, 0.0, 0.0);
            RL_.lower_leg.setOrigin(0.0, 0.0, -0.2, 0.0, 0.0, 0.0);
            RL_.foot.setOrigin(0.0, 0.0, -0.2, 0.0, 0.0, 0.0);

            legs[0] = &FR_;
            legs[1] = &FL_;
            legs[2] = &RR_;
            legs[3] = &RL_;
        }

};

#endif