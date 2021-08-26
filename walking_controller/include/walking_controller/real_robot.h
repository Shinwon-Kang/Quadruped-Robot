#ifndef REAL_ROBOT_H
#define REAL_ROBOT_H

#include <ros/ros.h>
#include <string>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "geometry_msgs/Twist.h"
#include "Eigen/Dense"
#include "array"

#include "a1.h"
#include "a1_components.h"
#include "body_controller.h"
#include "leg_controller.h"
#include "phase_signal_generator.h"
#include "kinematics.h"

#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

class RealRobot {
    private:
        A1 a1_base_;
        BodyController body_controller_;
        LegController leg_controller_;
        Kinematics kinematics_;

        unitree_legged_msgs::LowCmd send_low_ros_;
        unitree_legged_msgs::LowState recv_low_ros_;
    
        ros::Subscriber cmd_vel_subscriber_;
        Velocities req_vel_;
        Pose req_pose_;

        // void* update_loop(void* param);

        template<typename TCmd, typename TState, typename TLCM> int mainHelper(TLCM &roslcm);
        void matrixInitI_(std::array<Eigen::Matrix4f, 4>& m, int size);
        void cmdVelCallback_(const geometry_msgs::Twist::ConstPtr& msg);

    public:
        RealRobot(ros::NodeHandle *nh, ros::NodeHandle *pnh);
};

#endif