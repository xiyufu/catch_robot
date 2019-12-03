#include <cstdlib>

#include "visualization_msgs/Marker.h"
#include "planners.hpp"
#include "utility.hpp"
#include "running_robot.hpp"

// global variable.
nav_msgs::OccupancyGrid gEnvMap;
geometry_msgs::Pose gCurrentGoalPose;
bool gMapReady = false;

void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    gEnvMap.header = msg->header;
    gEnvMap.info = msg->info;
    gEnvMap.data = msg->data;

    gMapReady = true;
}

void GoalPoseCallback(const geometry_msgs::Pose::ConstPtr& msg){
    gCurrentGoalPose.position.x = msg->position.x;
    gCurrentGoalPose.position.y = msg->position.y;
}

int main(int argc ,char** argv){
    ros::init(argc, argv, "catch_robot");
    ros::NodeHandle n;
    ros::Subscriber map_sub = n.subscribe("basic_map", 1, MapCallback);
    ros::Subscriber goal_sub = n.subscribe("goal_pose", 1, GoalPoseCallback);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("res_path", 1);
    // ros::Publisher start_pose_pub = n.advertise<visualization_msgs::Marker>("start_pose", 1);
    // ros::Publisher goal_pose_pub = n.advertise<visualization_msgs::Marker>("goal_pose", 1);

    ros::Rate loop_rate(10); 

    nav_msgs::OccupancyGrid envmap;

    ROS_INFO("Waiting for map");
    while(!gMapReady){
        // Only update the map once, let's have a try first
        ros::spinOnce();
        loop_rate.sleep();
    }

    envmap = gEnvMap;
    geometry_msgs::Pose start_pose;
    geometry_msgs::Pose goal_pose = gCurrentGoalPose;

    start_pose.position.x = 1900;
    start_pose.position.y = 1700;

    point_robot_planner::AStarPlanner planner(envmap, start_pose, goal_pose);
    // on my machine (i7-9750H, RTX2060, 16GB ram), it takes about 0.4s for 1e5 iterations
    int max_iter = 10000000;
    planner.SetMaxIter(max_iter);

    PointRobot tracking_robot(start_pose, start_pose, &envmap);

    bool res_flag = false;
    bool caught_target = false;
    while(ros::ok() && !caught_target){
        ros::spinOnce();
        
        planner.SetGoalPose(goal_pose);
        planner.SetStartPose(start_pose);
        planner.RefreshNodes(); 
        {
        SimpleTimer planner_timer;
        res_flag = planner.MakePlan();
        }
        if(res_flag){
            ROS_INFO("Planning successed");
            if (start_pose.position.x == goal_pose.position.x && start_pose.position.y == goal_pose.position.y){
                caught_target = true;
            }
        }
        else{
            ROS_INFO("No solution found within the maximum iteration (%d), publishing what we have", max_iter);
        }

        const int current_action_id = planner.GetPlanAction()->back();
        const nav_msgs::Path* res_plan = planner.GetPlan();

        tracking_robot.StepForward(current_action_id, 8);

        // start_pose = tracking_robot.GetCurrentPose();
        start_pose = res_plan->poses.back().pose;
        goal_pose = gCurrentGoalPose;

        double marker_scale = 1 > envmap.info.width*0.01 ? 1 : envmap.info.width*0.01;
        visualization_msgs::Marker start_marker = AssmbleMarker(start_pose, 0, marker_scale);
        
        path_pub.publish(*res_plan);
        // start_pose_pub.publish(start_marker);

        //loop_rate.sleep();
    }

    if (caught_target){
        std::cout << "Got it" << std::endl;
    }

    return 0;
}