#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point32.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/WrenchStamped.h"

#include "Eigen/Dense"

#include "a1.h"
#include "a1_components.h"
#include "body_controller.h"
#include "leg_controller.h"
#include "phase_signal_generator.h"
#include "kinematics.h"
#include "pose_optimizer.h"

#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/ContactsStamped.h"

#include "jsk_recognition_msgs/PolygonArray.h"

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

    ros::Publisher optim_pose;
    ros::Publisher servo_pub_[12];
    ros::Publisher foot_contacts_pub_;

    ros::Publisher foot_polygon_;
    ros::Publisher cog_vis_pub_;

    ros::Subscriber footForce_sub_[4];
    float footForce_[4];        

    unitree_legged_msgs::LowCmd lowCmd;

    PoseOptimizer pose_optimizer_;


    // PID Controller
    // float kp=0.15, ki=0.02, kd=0.002;
    float kp=0.10, ki=0.08, kd=0.0;
    Eigen::Vector2f desired_roll_pitch;
    Eigen::Vector2f I_term;
    Eigen::Vector2f last_error;
    float max_I=0.1;
    ros::Time last_send_time;
    /////////////////////////////////////

    void controlLoop_(const ros::TimerEvent& event);
    void matrixInit_(std::array<Eigen::Matrix4f, 4>& m, int size);

    void centerOfGravity_(geometry_msgs::Point &center_of_gravity,
                          std::array<Eigen::Matrix4f, 4> target_foot_positions,
                          int contacts_n, 
                          bool foot_contacts[4]);

    geometry_msgs::Point centerOfGravityTriangle_(Eigen::Matrix4f p1, 
                                                  Eigen::Matrix4f p2, 
                                                  Eigen::Matrix4f p3);

    geometry_msgs::Point centerOfGravitySquare_(Eigen::Matrix4f p1, 
                                                Eigen::Matrix4f p2, 
                                                Eigen::Matrix4f p3,
                                                Eigen::Matrix4f p4);

    void cmdVelCallback_(const geometry_msgs::Twist::ConstPtr& msg);
    void ImuCallback_(const sensor_msgs::Imu::ConstPtr& msg);

    void publishJoints_(std::vector<double> target_joins);
    void publishJoints_(float target_joints[12]);

    int publishFootContacts_(bool (&foot_contacts)[4]);

    void FRfootCallback(const geometry_msgs::WrenchStamped& msg);
    void FLfootCallback(const geometry_msgs::WrenchStamped& msg);
    void RRfootCallback(const geometry_msgs::WrenchStamped& msg);
    void RLfootCallback(const geometry_msgs::WrenchStamped& msg);

    public:
        Controller(ros::NodeHandle *nh, ros::NodeHandle *pnh);
};

#endif