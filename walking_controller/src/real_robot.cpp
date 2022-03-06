#include "walking_controller/real_robot.h"

using namespace UNITREE_LEGGED_SDK;

PhaseSignalGenerator::Time rosTimeToRobotTimeForReal(const ros::Time& time) {
    return time.toNSec() / 1000ul;
}

RealRobot::RealRobot(ros::NodeHandle *nh, ros::NodeHandle *pnh):
    body_controller_(a1_base_),
    leg_controller_(a1_base_, rosTimeToRobotTimeForReal(ros::Time::now())),
    kinematics_(a1_base_)
{
    cmd_vel_subscriber_ = nh->subscribe("/cmd_vel/smooth", 1, &RealRobot::cmdVelCallback_, this);

    std::string robot_name = "a1";
    UNITREE_LEGGED_SDK::LeggedType rname;
    rname = UNITREE_LEGGED_SDK::LeggedType::A1;

    send_low_ros_.levelFlag = 0xff;
    for(int i = 0; i < 12; i ++) {
        send_low_ros_.motorCmd[i].mode = 0x0A;
        send_low_ros_.motorCmd[i].q = 2.146E+9f;
        send_low_ros_.motorCmd[i].Kp = 0;
        send_low_ros_.motorCmd[i].dq = 16000.0f;
        send_low_ros_.motorCmd[i].Kd = 0;
        send_low_ros_.motorCmd[i].tau = 0;
    }

    UNITREE_LEGGED_SDK::LCM roslcm(LOWLEVEL);
    mainHelper<UNITREE_LEGGED_SDK::LowCmd, UNITREE_LEGGED_SDK::LowState, UNITREE_LEGGED_SDK::LCM>(roslcm);
}

template<typename TLCM>
void* update_loop(void* param) {
    TLCM *data = (TLCM*)param;
    while(ros::ok) {
        data->Recv();
        usleep(2000);
    }
}

template<typename TCmd, typename TState, typename TLCM>
int RealRobot::mainHelper(TLCM &roslcm) {
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Pree Enter to continue.." << std::endl;
    std::cin.ignore();

    ros::NodeHandle n;
    ros::Rate loop_rate(500);

    long motiontime = 0;
    int rate_count = 0;
    int sin_count = 0;

    float qInit[3] = {0};
    float qDes[3] = {0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float Kp[3] = {0};
    float Kd[3] = {0};

    TCmd SendLowLCM = {0};
    TState RecvLowLCM = {0};

    // unitree_legged_msgs::LowCmd SendLowROS;
    // unitree_legged_msgs::LowState RecvLowROS;

    bool initiated_flag = false;
    int count = 0;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    while(ros::ok()) {
        roslcm.Get(RecvLowLCM);
        recv_low_ros_ = ToRos(RecvLowLCM); // Subscribe

        if(initiated_flag == true) {
            // control algorithm
            float target_joint_position[12];
            std::array<Eigen::Matrix4f, 4> target_foot_positions;
            matrixInitI_(target_foot_positions, 4);

            // body_controller_.run(target_foot_positions, req_pose_);
            // leg_controller_.run(target_foot_positions, req_vel_, rosTimeToRobotTime(ros::Time::now()));
            // kinematics_.inverse(target_joint_position, target_foot_positions, pose);


            // motor input
            for(int i = 0; i < 4; i++) {
                send_low_ros_.motorCmd[i*3+0].Kp = 200;
                send_low_ros_.motorCmd[i*3+0].dq = 0;
                send_low_ros_.motorCmd[i*3+0].Kd = 10;
                send_low_ros_.motorCmd[i*3+0].tau = 0;

                send_low_ros_.motorCmd[i*3+1].Kp = 200;
                send_low_ros_.motorCmd[i*3+1].dq = 0;
                send_low_ros_.motorCmd[i*3+1].Kd = 10;
                send_low_ros_.motorCmd[i*3+1].tau = 0;

                send_low_ros_.motorCmd[i*3+2].Kp = 200;
                send_low_ros_.motorCmd[i*3+1].dq = 0;
                send_low_ros_.motorCmd[i*3+1].Kd = 10;
                send_low_ros_.motorCmd[i*3+1].tau = 0;
            }

            for(int i = 0; i < 12; i++) {
                send_low_ros_.motorCmd[i].q = target_joint_position[i];
            }
        }

        SendLowLCM = ToLcm(send_low_ros_, SendLowLCM);
        roslcm.Send(SendLowLCM); // publish

        ros::spinOnce();
        loop_rate.sleep();

        count++;
        if(count > 10) {
            count = 10;
            initiated_flag = true;
        }
    }
}

void RealRobot::matrixInitI_(std::array<Eigen::Matrix4f, 4>& m, int size) {
    Eigen::Matrix4f m_;
    m_ << 1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0;
    for(int i = 0; i < size; i++)
        m[i] = m_;
}

void RealRobot::cmdVelCallback_(const geometry_msgs::Twist::ConstPtr& msg) {
    req_vel_.linear.x = msg->linear.x;
    req_vel_.linear.y = msg->linear.y;
    req_vel_.angular.z = msg->angular.z;
}

// int main(int argc, char *argv[]) {
//     ros::init(argc, argv, "position_ros_mode");
//     std::string firmwork;
//     ros::param::get("/firmwork", firmwork);

//     std::string robot_name;
//     UNITREE_LEGGED_SDK::LeggedType rname;
//     ros::param::get("/robot_name", robot_name);

//     rname = UNITREE_LEGGED_SDK::LeggedType::A1;

//     UNITREE_LEGGED_SDK::LCM roslcm(LOWLEVEL);
//     mainHelper<UNITREE_LEGGED_SDK::LowCmd, UNITREE_LEGGED_SDK::LowState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
// }
