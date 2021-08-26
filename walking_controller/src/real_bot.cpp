#include "ros/ros.h"
#include "walking_controller/real_robot.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "real_robot");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    RealRobot control(&nh, &nh_private);

    ros::spin();
    return 0;
}