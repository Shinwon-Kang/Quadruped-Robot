#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"
#include <Eigen/Dense>

#include "a1.h"
#include "a1_components.h"
#include "body_controller.h"
#include "leg_controller.h"
#include "phase_signal_generator.h"
#include "kinematics.h"

#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/MotorCmd.h"

class Controller {
    ros::Subscriber cmd_vel_subscriber_;
    ros::Subscriber imu_subscriber_;

    ros::Timer loop_func_;

    Velocities req_vel_;
    Pose req_pose_;

    A1 a1_base_;
    BodyController body_controller_;
    LegController leg_controller_;
    Kinematics kinematics_;

    ros::Publisher servo_pub[12];
    unitree_legged_msgs::LowCmd lowCmd;

    void controlLoop_(const ros::TimerEvent& event);

    void cmdVelCallback_(const geometry_msgs::Twist::ConstPtr& msg);
    void ImuCallback_(const sensor_msgs::Imu::ConstPtr& msg);

    void matrixInit_(std::array<Eigen::Matrix4f, 4>& m, int size);
    void publishJoints_(float target_joints[12]);

    public:
        Controller(ros::NodeHandle *nh, ros::NodeHandle *pnh);
};

#endif