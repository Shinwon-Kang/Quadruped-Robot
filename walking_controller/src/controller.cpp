#include <walking_controller/controller.h>

PhaseSignalGenerator::Time rosTimeToRobotTime(const ros::Time& time) {
    return time.toNSec() / 1000ul;
}

Controller::Controller(ros::NodeHandle *nh, ros::NodeHandle *pnh):
    body_controller_(a1_base_),
    leg_controller_(a1_base_, rosTimeToRobotTime(ros::Time::now())),
    kinematics_(a1_base_)
{
    lowState_subscriber_ = nh->subscribe("/a1_gazebo/lowState/state", 1, &Controller::lowStateCallback_, this);

    cmd_vel_subscriber_ = nh->subscribe("/cmd_vel/smooth", 1, &Controller::cmdVelCallback_, this);
    imu_subscriber_ = nh->subscribe("/trunk_imu", 1, &Controller::ImuCallback_, this);

    footForce_sub_[0] = nh->subscribe("/visual/FR_foot_contact/the_force", 1, &Controller::FRfootCallback, this);
    footForce_sub_[1] = nh->subscribe("/visual/FL_foot_contact/the_force", 1, &Controller::FLfootCallback, this);
    footForce_sub_[2] = nh->subscribe("/visual/RR_foot_contact/the_force", 1, &Controller::RRfootCallback, this);
    footForce_sub_[3] = nh->subscribe("/visual/RL_foot_contact/the_force", 1, &Controller::RLfootCallback, this);

    servo_pub_[0] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_hip_controller/command", 1);
    servo_pub_[1] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_thigh_controller/command", 1);
    servo_pub_[2] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_calf_controller/command", 1);
    servo_pub_[3] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_hip_controller/command", 1);
    servo_pub_[4] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_thigh_controller/command", 1);
    servo_pub_[5] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_calf_controller/command", 1);
    servo_pub_[6] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_hip_controller/command", 1);
    servo_pub_[7] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_thigh_controller/command", 1);
    servo_pub_[8] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_calf_controller/command", 1);
    servo_pub_[9] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_hip_controller/command", 1);
    servo_pub_[10] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_thigh_controller/command", 1);
    servo_pub_[11] = nh->advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_calf_controller/command", 1);

    foot_contacts_pub_ = nh->advertise<unitree_legged_msgs::ContactsStamped>("foot_contacts", 1);
    optim_pose = nh->advertise<geometry_msgs::Pose>("optim_pose", 1);

    // standup();

    double loop_rate = 200.0;
    loop_func_ = pnh->createTimer(ros::Duration(1 / loop_rate),
                                  &Controller::controlLoop_,
                                  this);
    
    req_pose_.position.z = 0.28;

    desired_roll_pitch.setZero();
    I_term.setZero();
    last_error.setZero();
    last_send_time = ros::Time::now();
}

void Controller::standup() {
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

    double target_pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                             0.0, 0.67, -1.3, -0.0, 0.67, -1.3};

    double duration = 20000.0;
    double pos[12] ,lastPos[12], percent=0;

    // TODO: motor의 처음 q를 불러와야됨

    for(int j=0; j<12; j++) lastPos[j] = motorQstate[j];
    for(int i=1; i<=duration; i++){
        if(!ros::ok()) break;
        percent = (double)i/duration;
        for(int j=0; j<12; j++){
            lowCmd.motorCmd[j].q = lastPos[j]*(1-percent) + target_pos[j]*percent;
        }

        for(int m=0; m<12; m++){
            servo_pub_[m].publish(lowCmd.motorCmd[m]);
        }
        usleep(1000);
    }

    init_standup = true;
}

void Controller::controlLoop_(const ros::TimerEvent& event) {
    if(!init_standup)
        return;

    float target_joint_position[12];
    std::array<Eigen::Matrix4f, 4> target_foot_positions;
    matrixInit_(target_foot_positions, 4);

    bool foot_contacts[4];
    geometry_msgs::Point center_of_gravity;

    // 1. PoseCommand
    body_controller_.run(target_foot_positions, req_pose_);

    // 2. VelocityCommand
    leg_controller_.run(target_foot_positions, req_vel_, rosTimeToRobotTime(ros::Time::now()));

    int contacts_n = publishFootContacts_(foot_contacts);

    // 4. Pose optimizer        
    // std::cout << "target_foot_positions: " << std::endl << target_foot_positions[2](0, 3) 
    //                                        << std::endl << target_foot_positions[2](1, 3)
    //                                        << std::endl << target_foot_positions[2](2, 3) << std::endl;

    // target_foot_positions[1](0, 3) = 0.01; target_foot_positions[1](1, 3) = -0.0838; target_foot_positions[1](2, 3) = -0.2; //FR
    Eigen::VectorXd pose = pose_optimizer_.optimize(target_foot_positions, foot_contacts, req_pose_.orientation.roll, req_pose_.orientation.pitch, req_pose_.orientation.yaw);
    // std::cout << "POSE: " << pose(0) << ", " << pose(1) << ", " << pose(2) << std::endl;

    // NEW  AND  IMU PID Controller /////////////////
    Eigen::Vector2f error;
    error[0] = desired_roll_pitch[0] - req_pose_.orientation.roll;
    error[1] = desired_roll_pitch[1] - req_pose_.orientation.pitch;

    ros::Time t_now = ros::Time::now();
    double step = (t_now - last_send_time).toSec();

    I_term += error * step;

    for(int i = 0; i < 2; i++){
        if(I_term[i] < -max_I)
            I_term[i] = -max_I;
        else if(I_term[i] > max_I)
            I_term[i] = max_I;
    }

	Eigen::Vector2f D_term;
    if(step == 0)
        D_term.setZero();
    else
        D_term = (error - last_error) / step;
    last_send_time = t_now;
    last_error = error;

    Eigen::Vector2f ret;
    ret = error * kp;
    ret += I_term * ki;
    ret += D_term * kd;

    float roll_comp = -ret[0];
    float pitch_comp = -ret[1];
    Eigen::Matrix3f rot = rotxyz(roll_comp, pitch_comp, 0);

    // 3. Inverse
    kinematics_.inverse(target_joint_position, target_foot_positions, pose);


    // 3. Inverse Test
    // Eigen::Matrix<float, 3, 4> leg_positions;
    // leg_positions << target_foot_positions[1](0, 3)+0.1805, target_foot_positions[0](0, 3)+0.1805, target_foot_positions[3](0, 3)-0.1805, target_foot_positions[2](0, 3)-0.1805,
    //                  target_foot_positions[1](1, 3)-0.047, target_foot_positions[0](1, 3)+0.047, target_foot_positions[3](1, 3)-0.047, target_foot_positions[2](1, 3)+0.047,
    //                  target_foot_positions[1](2, 3), target_foot_positions[0](2, 3), target_foot_positions[3](2, 3), target_foot_positions[2](2, 3);

    // std::cout << target_foot_positions[0](1, 3) << ", " << target_foot_positions[1](1, 3) << ", " << target_foot_positions[2](1, 3) << ", " << target_foot_positions[3](1, 3) << std::endl;
    // leg_positions << target_foot_positions[0](0, 3)+0.1805, target_foot_positions[1](0, 3)+0.1805, target_foot_positions[2](0, 3)-0.1805, target_foot_positions[3](0, 3)-0.1805,
    //                  target_foot_positions[0](1, 3)+0.047, target_foot_positions[1](1, 3)-0.047, target_foot_positions[2](1, 3)+0.047, target_foot_positions[3](1, 3)-0.047,
    //                  target_foot_positions[0](2, 3), target_foot_positions[1](2, 3), target_foot_positions[2](2, 3), target_foot_positions[3](2, 3);

    // for(int i=0; i<4; i++) {
        // leg_positions.col(i) = rot * leg_positions.col(i);
    // }

    // std::cout << "leg_positions" << std::endl << leg_positions << std::endl;
    float dx = pose(0);
    float dy = pose(1);
    float dz = pose(2);
    std::cout << "Pose Optim: " << dx << ", " << dy << ", " << dz << std::endl;

    geometry_msgs::Pose po;
    geometry_msgs::Point p;
    p.x = dx;
    p.y = dy;
    p.z = dz;
    po.position = p;
    optim_pose.publish(po);
    
    // std::vector<double> test_angles = kinematics_.wholeBodyIK(leg_positions, dx, dy, 0.0, 0.0, 0.0, dz);
    // std::vector<double> test_angles = kinematics_.wholeBodyIK(leg_positions, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // 4. Publish
    publishJoints_(target_joint_position);
    // publishJoints_(test_angles);

    a1_base_.FL_.force(footForce_[0]);
    a1_base_.FR_.force(footForce_[1]);
    a1_base_.RL_.force(footForce_[2]);
    a1_base_.RR_.force(footForce_[3]);
}

void Controller::publishJoints_(std::vector<double> target_joints) {
    for(int i=0; i<4; i++){
        lowCmd.motorCmd[i*3+0].mode = 0x0A;
        lowCmd.motorCmd[i*3+0].Kp = 105;
        lowCmd.motorCmd[i*3+0].dq = 0;
        lowCmd.motorCmd[i*3+0].Kd = 4.5;
        lowCmd.motorCmd[i*3+0].tau = 0;
        lowCmd.motorCmd[i*3+1].mode = 0x0A;
        lowCmd.motorCmd[i*3+1].Kp = 105;
        lowCmd.motorCmd[i*3+1].dq = 0;
        lowCmd.motorCmd[i*3+1].Kd = 4.5;
        lowCmd.motorCmd[i*3+1].tau = 0;
        lowCmd.motorCmd[i*3+2].mode = 0x0A;
        lowCmd.motorCmd[i*3+2].Kp = 105;
        lowCmd.motorCmd[i*3+2].dq = 0;
        lowCmd.motorCmd[i*3+2].Kd = 4.5;
        lowCmd.motorCmd[i*3+2].tau = 0;
    }

    for(int i=0; i<12; i++){
        lowCmd.motorCmd[i].q = target_joints[i];
    }

    for(unsigned int i = 0; i < 12; i ++) {
        servo_pub_[i].publish(lowCmd.motorCmd[i]);
    }
}

void Controller::publishJoints_(float target_joints[12]) {
    for(int i=0; i<4; i++){
        lowCmd.motorCmd[i*3+0].mode = 0x0A;
        lowCmd.motorCmd[i*3+0].Kp = 200;
        lowCmd.motorCmd[i*3+0].dq = 0;
        lowCmd.motorCmd[i*3+0].Kd = 10;
        lowCmd.motorCmd[i*3+0].tau = 0;
        lowCmd.motorCmd[i*3+1].mode = 0x0A;
        lowCmd.motorCmd[i*3+1].Kp = 200;
        lowCmd.motorCmd[i*3+1].dq = 0;
        lowCmd.motorCmd[i*3+1].Kd = 10;
        lowCmd.motorCmd[i*3+1].tau = 0;
        lowCmd.motorCmd[i*3+2].mode = 0x0A;
        lowCmd.motorCmd[i*3+2].Kp = 200;
        lowCmd.motorCmd[i*3+2].dq = 0;
        lowCmd.motorCmd[i*3+2].Kd = 10;
        lowCmd.motorCmd[i*3+2].tau = 0;
    }

    for(int i=0; i<12; i++){
        lowCmd.motorCmd[i].q = target_joints[i];
    }
    // lowCmd.motorCmd[0].q = target_joints[3];
    // lowCmd.motorCmd[1].q = target_joints[4];
    // lowCmd.motorCmd[2].q = target_joints[5];

    // lowCmd.motorCmd[3].q = target_joints[0];
    // lowCmd.motorCmd[4].q = target_joints[1];
    // lowCmd.motorCmd[5].q = target_joints[2];

    // lowCmd.motorCmd[6].q = target_joints[9];
    // lowCmd.motorCmd[7].q = target_joints[10];
    // lowCmd.motorCmd[8].q = target_joints[11];

    // lowCmd.motorCmd[9].q = target_joints[6];
    // lowCmd.motorCmd[10].q = target_joints[7];
    // lowCmd.motorCmd[11].q = target_joints[8];

    for(unsigned int i = 0; i < 12; i ++) {
        servo_pub_[i].publish(lowCmd.motorCmd[i]);
    }
}

int Controller::publishFootContacts_(bool (&foot_contacts)[4]) {
    unitree_legged_msgs::ContactsStamped contacts_msg;
    contacts_msg.header.stamp = ros::Time::now();
    contacts_msg.contacts.resize(4);

    int contact_n = 0;
    for(size_t i=0; i<4; i++) {
        bool is_contact = a1_base_.legs[i]->gait_phase();

        contacts_msg.contacts[i] = is_contact;
        foot_contacts[i] = is_contact;

        if(is_contact) contact_n++;
    }
    foot_contacts_pub_.publish(contacts_msg);

    return contact_n;
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

    // req_pose_.position.x = msg->position.x;
    // req_pose_.position.y = msg->position.y;
    // req_pose_.position.z = msg->position.z;
}

void Controller::lowStateCallback_(const unitree_legged_msgs::LowState::ConstPtr& msg) {
    for(int i = 0; i < 12; i++) {
        motorQstate[i] = msg->motorState[i].q;
    }
    init_motor = true;
}

void Controller::FRfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    footForce_[0] = msg.wrench.force.z;
}

void Controller::FLfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    footForce_[1] = msg.wrench.force.z;
}

void Controller::RRfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    footForce_[2] = msg.wrench.force.z;
}

void Controller::RLfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    footForce_[3] = msg.wrench.force.z;
}
