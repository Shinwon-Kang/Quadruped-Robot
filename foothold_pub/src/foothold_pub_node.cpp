
// INCLUDE grid_map libraries
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/ProcessFile.h>
#include <grid_map_msgs/SetGridMap.h>

#include <tf/transform_listener.h>

#include <cmath>
#include <algorithm>

tf::StampedTransform transform;

grid_map::GridMap map_from_topic;
grid_map::GridMap map_to_topic({"elevation"});

grid_map::Position robot_position;
double robot_roll, robot_pitch, robot_yaw;

// Set Gridmap
grid_map::Length length(2.0, 2.0);
grid_map::Position position(0.0, 0.0);
double resolution = 0.05;

void elevationMapCallback(const grid_map_msgs::GridMap& msg) {
    grid_map::GridMapRosConverter::fromMessage(msg, map_from_topic);

    grid_map::Position p(0, 0);
    // double max = -100;

    for (int i=0; i<map_to_topic.getSize()(1); i++) for (int j=0; j<map_to_topic.getSize()(0); j++) {
        grid_map::Index idx(i, j);
        map_to_topic.getPosition(idx, p);

        if (map_from_topic.isInside(p)) {
            // map_to_topic.at("elevation", idx) = map_from_topic.atPosition("elevation", p);
            double value = 1 - map_from_topic.atPosition("elevation", p) / 0.2;

            if(value > 0.4) {
                map_to_topic.at("elevation", idx) = 1;
            } else {
                map_to_topic.at("elevation", idx) = 0;
            }
        } else
            ROS_ERROR("ERROR Position: %f %f", p(0), p(1));

        // double v = map_from_topic.atPosition("elevation", p);
        // if(!std::isnan(v) && v > max) {
            // max = v;
        // }
    }
    // ROS_INFO("Max value %f", max);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "foothold_pub");
    ros::NodeHandle n;

    ros::Publisher elevation_map_pub = n.advertise<grid_map_msgs::GridMap>("foothold_map", 1);
    ros::Subscriber elevation_map_sub = n.subscribe("/elevation_mapping/elevation_map", 1, elevationMapCallback);

    tf::TransformListener listener;
    ros::Rate rate(100.0);

    map_to_topic.setGeometry(length, resolution, position);
    grid_map_msgs::GridMap message;
    map_to_topic.setBasicLayers({"elevation"});
    map_to_topic.setFrameId("/odom");

    while(n.ok()) {
        try {
            listener.waitForTransform("/odom", "/base_footprint",
                                      ros::Time(0), ros::Duration(5.0));
            listener.lookupTransform("/odom", "/base_footprint",
                                     ros::Time(0), transform);
        } catch(tf::TransformException &e) {
            continue;
        }
        robot_position.x() = transform.getOrigin().x();
        robot_position.y() = transform.getOrigin().y();

        // tf::Quaternion q(
        //     transform.getRotation().x(),
        //     transform.getRotation().y(),
        //     transform.getRotation().z(),
        //     transform.getRotation().w()
        // );
        // tf::Matrix3x3 m(q);
        // m.getRPY(robot_roll, robot_pitch, robot_yaw);

        grid_map::Position position_(robot_position.x(), robot_position.y());
        map_to_topic.setPosition(position_);

        grid_map::GridMapRosConverter::toMessage(map_to_topic, message);
        elevation_map_pub.publish(message);
        ros::spinOnce();
    }

    return 0;
}