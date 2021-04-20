#include "ros/ros.h"
#include <controller.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_node");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    Controller control(&nh, &nh_private);

    ros::spin();
    return 0;
}