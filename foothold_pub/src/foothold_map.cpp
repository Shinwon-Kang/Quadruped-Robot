#include "foothold_map.h"

FootHoldMap::FootHoldMap()
    : map_length_(1.0, 1.0),
      map_position_(0.0, 0.0),
      map_resolution_(0.05),
      foothold_map_({"elevation"}) {

    foothold_map_.setGeometry(map_length_, map_resolution_, map_position_);
    foothold_map_.setBasicLayers({"elevation"});
    foothold_map_.setFrameId("/odom");
}

void FootHoldMap::createFootHoldMap(const grid_map::GridMap map) {
    grid_map::Position p;

    for (int i=0; i<map.getSize()(1); i++) for (int j=0; j<map.getSize()(0); j++) {
        grid_map::Index idx(i, j);
        foothold_map_.getPosition(idx, p);

        if (map.isInside(p))
            foothold_map_.at("elevation", idx) = map.atPosition("elevation", p);
        else
            ROS_ERROR("ERROR Position: %f %f", p(0), p(1));
    }
}

const grid_map::GridMap FootHoldMap::getGridMap() const {
    return foothold_map_;
}