#ifndef CATCH_ROBOT_POINT_UTILITY
#define CATCH_ROBOT_POINT_UTILITY

#include <chrono>
#include <ctime>
#include <iostream>

#include "planners.hpp"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"

class SimpleTimer{
    std::chrono::time_point<std::chrono::system_clock> init_time;
public:
    SimpleTimer(){
        init_time = std::chrono::system_clock::now();
    }
    ~SimpleTimer(){
        auto end_time = std::chrono::system_clock::now();
        std::chrono::duration<double> life_duration = end_time - init_time; 
        std::cout << "Time cost: " << life_duration.count() << "s" << std::endl;
    }
};

bool IsOccupied(const nav_msgs::OccupancyGrid& envmap, const geometry_msgs::Pose& pose);

double CalculateDistance(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2);
// type == 0: start pose, type == 1: goal pose
visualization_msgs::Marker AssmbleMarker(geometry_msgs::Pose p, const int type, const double marker_scale);




#endif