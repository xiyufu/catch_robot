#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"
#include "utility.hpp"
#include "planners.hpp"
#include "running_robot.hpp"

nav_msgs::OccupancyGrid gMap;
bool gMapFlag = false;

void MapUpdateCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    gMap.header = msg->header;
    gMap.info = msg->info;
    gMap.data = msg->data;
    gMapFlag = true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "running_robot");
    ros::NodeHandle n;
    ros::Publisher robot_pub = n.advertise<geometry_msgs::Pose>("goal_pose", 1);
    ros::Publisher goal_marker_pub = n.advertise<visualization_msgs::Marker>("goal_marker", 1);
    ros::Publisher start_marker_pub = n.advertise<visualization_msgs::Marker>("start_marker", 1);
    ros::Subscriber map_sub = n.subscribe("basic_map", 1, MapUpdateCallback);
    ros::Rate loop_rate(5);

    ROS_INFO("Waiting for map");
    while (ros::ok()){
        ros::spinOnce();
        if (!gMapFlag){
            loop_rate.sleep();
        }
        else{
            break;
        }
        
    }
    ROS_INFO("Map received");

    nav_msgs::OccupancyGrid envmap = gMap;
    // The robot is initialized at the goal_pose and will run away from the start_pose
    geometry_msgs::Pose start_pose;
    geometry_msgs::Pose goal_pose;

    do{
        std::srand(ros::Time::now().nsec);
        start_pose.position.x = std::rand() % envmap.info.width;
        start_pose.position.y = std::rand() % envmap.info.height;
        goal_pose.position.x = std::rand() % envmap.info.width;
        goal_pose.position.y = std::rand() % envmap.info.height;
    }
    while(IsOccupied(envmap, start_pose) || IsOccupied(envmap, goal_pose) 
            || CalculateDistance(start_pose, goal_pose) < envmap.info.width/3.0);

    visualization_msgs::Marker init_marker_robot;
    init_marker_robot.header.frame_id = "map";
    init_marker_robot.ns = "plot_res";
    init_marker_robot.header.stamp = ros::Time::now();
    init_marker_robot.pose = start_pose;
    init_marker_robot.id = 1;
    init_marker_robot.action = visualization_msgs::Marker::ADD;
    init_marker_robot.type = visualization_msgs::Marker::SPHERE;
    init_marker_robot.color.a = 1;
    init_marker_robot.color.r = 0;
    init_marker_robot.color.g = 1;
    init_marker_robot.color.b = 0;
    double marker_scale = 1 > envmap.info.width*0.01 ? 1 : envmap.info.width*0.01;
    init_marker_robot.scale.x = marker_scale;
    init_marker_robot.scale.y = marker_scale;
    init_marker_robot.scale.z = marker_scale;
    
    PointRobot escaper(goal_pose, start_pose, &envmap);

    ROS_INFO("Robot running");
    while(ros::ok()){
        geometry_msgs::Pose cur_pose = escaper.GetCurrentPose();
        robot_pub.publish(cur_pose);
        
        visualization_msgs::Marker marker_robot;
        marker_robot.header.frame_id = "map";
        marker_robot.ns = "plot_res";
        marker_robot.header.stamp = ros::Time::now();
        marker_robot.pose = cur_pose;
        marker_robot.id = 0;
        marker_robot.action = visualization_msgs::Marker::ADD;
        marker_robot.type = visualization_msgs::Marker::SPHERE;
        marker_robot.color.a = 1;
        marker_robot.color.r = 1;
        marker_robot.color.g = 0;
        marker_robot.color.b = 0;
        double marker_scale = 1 > envmap.info.width*0.01 ? 1 : envmap.info.width*0.01;
        marker_robot.scale.x = marker_scale;
        marker_robot.scale.y = marker_scale;
        marker_robot.scale.z = marker_scale;

        start_marker_pub.publish(init_marker_robot);
        goal_marker_pub.publish(marker_robot);

        int action_id = escaper.StepForward();

        ROS_INFO("Action id is: %d", action_id);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}