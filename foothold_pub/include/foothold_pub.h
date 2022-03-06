#ifndef FOOTHOLD_PUB_H
#define FOOTHOLD_PUB_H

#include "ros/ros.h"

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/ProcessFile.h>
#include <grid_map_msgs/SetGridMap.h>

#include <tf/transform_listener.h>

#include <boost/thread.hpp>

#include "champ_msgs/ContactsStamped.h"
#include "foothold_map.h"

class FootHoldPub {
    grid_map::GridMap map_from_topic_;
    grid_map::GridMap map_to_topic_;

    tf::StampedTransform transform_;
    tf::TransformListener listener_;

    // FL, FR, RL, RR
    grid_map::Position base_footprint_position_;
    std::array<grid_map::Position, 4> foot_positions_;
    std::array<bool, 4> foot_contacts_;
    
    ros::NodeHandle nh_;

    ros::Subscriber foot_contact_sub_;
    ros::Subscriber elevation_map_sub_;

    ros::Publisher map_pub_;

    FootHoldMap fhm;

    void tfListenerThread(std::string parent_tf, std::string child_tf);
    void tfListenerThread(std::string parent_tf, std::string child_tf, int idx);

    // Subscribe
    void run();
    void elevationMapCallback(const grid_map_msgs::GridMap& msg);
    void footContactsCallback(const champ_msgs::ContactsStamped msg);

    public:
        FootHoldPub();
};

#endif