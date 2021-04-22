#include "foothold_pub.h"

FootHoldPub::FootHoldPub()
    : base_footprint_position_(0.0, 0.0),
      map_to_topic_({"elevation"}) {

    grid_map::Length length(2.0, 2.0);
    grid_map::Position position(0.0, 0.0);
    double resolution = 0.05;
    map_to_topic_.setGeometry(length, resolution, position);
    map_to_topic_.setBasicLayers({"elevation"});
    map_to_topic_.setFrameId("/odom");

    grid_map::Position p(0.0, 0.0);

    // FL, FR, RL, RR
    for(int i=0; i<foot_contacts_.size(); i++) {
        foot_positions_[i] = p;
        foot_contacts_[i] = false;
    }

    // foot_contact_sub_ = nh_.subscribe("/foot_contacts", 1, &FootHoldPub::footContactsCallback, this);
    elevation_map_sub_ = nh_.subscribe("/elevation_mapping/elevation_map_raw", 1, &FootHoldPub::elevationMapCallback, this);
    map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("foothold_map", 1);

    run();
}

void FootHoldPub::run() {
    while(nh_.ok()) {
        std::cout << "@" << std::endl;
        try {
            std::cout << "@@" << std::endl;
            listener_.waitForTransform("/odom", "/base_footprint",
                                                ros::Time(0), ros::Duration(5.0));
            listener_.lookupTransform("/odom", "/base_footprint",
                                                ros::Time(0), transform_);

            base_footprint_position_.x() = transform_.getOrigin().x();
            base_footprint_position_.y() = transform_.getOrigin().x();
            std::cout << "@@@" << std::endl;

        } catch(tf::TransformException &e) {
            continue;
        }

        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(map_to_topic_, message);
        map_pub_.publish(message);
        ros::spinOnce();
    }
}

void FootHoldPub::tfListenerThread(std::string parent_tf, std::string child_tf, int idx) {
    while(nh_.ok()) {
        try {
            listener_.waitForTransform(parent_tf, child_tf,
                                                ros::Time(0), ros::Duration(5.0));
            listener_.lookupTransform(parent_tf, child_tf,
                                                ros::Time(0), transform_);

            foot_positions_[idx].x() = transform_.getOrigin().x();
            foot_positions_[idx].y() = transform_.getOrigin().x();
        } catch(tf::TransformException &e) {
            continue;
        }
    }
}

// Subscribe
void FootHoldPub::elevationMapCallback(const grid_map_msgs::GridMap& msg) {
    grid_map::GridMapRosConverter::fromMessage(msg, map_from_topic_);

    grid_map::Position p(0, 0);
    for (int i=0; i<map_from_topic_.getSize()(1); i++) for (int j=0; j<map_from_topic_.getSize()(0); j++) {
        grid_map::Index idx(i, j);
        map_to_topic_.getPosition(idx, p);

        if (map_from_topic_.isInside(p))
            map_to_topic_.at("elevation", idx) = map_from_topic_.atPosition("elevation", p);
        else
            ROS_ERROR("ERROR Position: %f %f", p(0), p(1));
    }
}

void FootHoldPub::footContactsCallback(const champ_msgs::ContactsStamped msg) {
    for (int i=0; i<msg.contacts.size(); i++) {
        if(msg.contacts[i]) {
            // fhm.createFootHoldMap(map_from_topic_);
            // grid_map::GridMap m = fhm.getGridMap();

            // m.move(base_footprint_position_);
            grid_map_msgs::GridMap message;
            // grid_map::GridMapRosConverter::toMessage(m, message);
            // map_pub_.publish(message);
        }
    }
}
