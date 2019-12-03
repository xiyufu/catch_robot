#include "utility.hpp"

bool IsOccupied(const nav_msgs::OccupancyGrid& envmap, const geometry_msgs::Pose& pose){
    int id_in_map = pose.position.y * envmap.info.width + pose.position.x;
    return (envmap.data[id_in_map] != 0);
}

double CalculateDistance(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2){
    double dist_sq = std::pow(p1.position.x - p2.position.x, 2);
    dist_sq += std::pow(p1.position.y - p2.position.y, 2);
    return std::sqrt(dist_sq);
}

visualization_msgs::Marker AssmbleMarker(geometry_msgs::Pose p, int type, const double marker_scale){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "plot_res";
    marker.id = type;
    marker.pose.position.x = p.position.x;
    marker.pose.position.y = p.position.y;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.color.a = 1;
    marker.color.r = type;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.scale.x = marker_scale;
    marker.scale.y = marker_scale;
    marker.scale.z = marker_scale;

    return marker;
}
