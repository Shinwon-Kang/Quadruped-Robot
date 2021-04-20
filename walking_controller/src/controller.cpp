#include <controller.h>

PhaseSignalGenerator::Time rosTimeToRobotTime(const ros::Time& time) {
    return time.toNSec() / 1000ul;
}

Controller::Controller(ros::NodeHandle *nh, ros::NodeHandle *pnh):
    body_controller_(a1_base_),
    leg_controller_(a1_base_, rosTimeToRobotTime(ros::Time::now())),
    kinematics_(a1_base_)
{
    cmd_vel_subscriber_ = nh->subscribe("/cmd_vel/smooth", 1, &Controller::cmdVelCallback_, this);
    // imu_subscriber_ = nh->subscribe("/imu/data", 1, &Controller::ImuCallback_, this);

    servo_pub[0] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_calf_controller/command", 1);

    double loop_rate = 200.0;
    loop_func_ = pnh->createTimer(ros::Duration(1 / loop_rate),
                                  &Controller::controlLoop_,
                                  this);
    
}

void Controller::controlLoop_(const ros::TimerEvent& event) {
    float target_joint_position[12];
    std::array<Eigen::Matrix4f, 4> target_foot_positions;
    matrixInit_(target_foot_positions, 4);

    bool foot_contacts[4];

    // 1. PoseCommand
    body_controller_.run(target_foot_positions, req_pose_);

    // 2. VelocityCommand
    leg_controller_.run(target_foot_positions, req_vel_, rosTimeToRobotTime(ros::Time::now()));
    // std::cout << "FR: " << target_foot_positions[0](0, 3) << ", " << target_foot_positions[0](1, 3) << ", " << target_foot_positions[0](2, 3) << std::endl;

    // 3. Inverse
    kinematics_.inverse(target_joint_position, target_foot_positions);
    // for(unsigned int i = 0; i < 12; i++) {
    //     std::cout << target_joint_position[i] << " ";
    // }
    // std::cout << std::endl;

    // 4. Publish
    publishJoints_(target_joint_position);
}

void Controller::publishJoints_(float target_joints[12]) {
    for(int i=0; i<4; i++){
        lowCmd.motorCmd[i*3+0].mode = 0x0A;
        lowCmd.motorCmd[i*3+0].Kp = 100;
        lowCmd.motorCmd[i*3+0].dq = 0;
        lowCmd.motorCmd[i*3+0].Kd = 4;
        lowCmd.motorCmd[i*3+0].tau = 0;
        lowCmd.motorCmd[i*3+1].mode = 0x0A;
        lowCmd.motorCmd[i*3+1].Kp = 100;
        lowCmd.motorCmd[i*3+1].dq = 0;
        lowCmd.motorCmd[i*3+1].Kd = 4;
        lowCmd.motorCmd[i*3+1].tau = 0;
        lowCmd.motorCmd[i*3+2].mode = 0x0A;
        lowCmd.motorCmd[i*3+2].Kp = 100;
        lowCmd.motorCmd[i*3+2].dq = 0;
        lowCmd.motorCmd[i*3+2].Kd = 4;
        lowCmd.motorCmd[i*3+2].tau = 0;
    }

    for(int i=0; i<12; i++){
        lowCmd.motorCmd[i].q = target_joints[i];
    }

    for(unsigned int i = 0; i < 12; i ++) {
        servo_pub[i].publish(lowCmd.motorCmd[i]);
    }
}

void Controller::matrixInit_(std::array<Eigen::Matrix4f, 4>& m, int size) {
    Eigen::Matrix4f m_;
    m_ << 1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0;
    for(int i = 0; i < size; i++)
        m[i] = m_;
}

void Controller::cmdVelCallback_(const geometry_msgs::Twist::ConstPtr& msg) {
    req_vel_.linear.x = msg->linear.x;
    req_vel_.linear.y = msg->linear.y;
    req_vel_.angular.z = msg->angular.z;
}

void Controller::ImuCallback_(const sensor_msgs::Imu::ConstPtr& msg) {
    // TODO: TEST

    tf::Quaternion q (
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    req_pose_.orientation.roll = roll;
    req_pose_.orientation.pitch = pitch;
    req_pose_.orientation.yaw = yaw;

    std::cout << roll << ", " << pitch << ", " << yaw << std::endl;

    // req_pose_.position.x = msg->position.x;
    // req_pose_.position.y = msg->position.y;
    // req_pose_.position.z = msg->position.z +  0.3;
}