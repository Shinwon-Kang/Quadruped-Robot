#ifndef FOOTHOLD_MAP_H
#define FOOTHOLD_MAP_H

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/ProcessFile.h>
#include <grid_map_msgs/SetGridMap.h>


class FootHoldMap {
    grid_map::Length map_length_;
    grid_map::Position map_position_;
    double map_resolution_;

    grid_map::GridMap foothold_map_;

    public:
        FootHoldMap();
        void createFootHoldMap(const grid_map::GridMap map);
        const grid_map::GridMap getGridMap() const;
};

#endif