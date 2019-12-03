#ifndef RUNNING_ROBOT
#define RUNNING_ROBOT

#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "utility.hpp"

// point robot running on 8 or 4 connected 2d grid
class PointRobot{
    geometry_msgs::Pose current_pose;
    geometry_msgs::Pose run_away_pose;
    const nav_msgs::OccupancyGrid* envmap;
    double current_distance;
    int no_movement_count = 0;

    // index = 1, 3, 5, 7 is only valid for 8-connected grid
    const int actions_x[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    const int actions_y[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const double actions_cost[8] {1, 1.414, 1, 1.414, 1, 1.414, 1, 1.414};

public:
    PointRobot(const geometry_msgs::Pose& init_pose, const geometry_msgs::Pose& run_away,
                const nav_msgs::OccupancyGrid* global_map)
    : envmap(global_map){
        current_pose = init_pose;
        run_away_pose = run_away;
        current_distance = CalculateDistance(init_pose, run_away_pose);
    }

    inline const geometry_msgs::Pose GetCurrentPose(){
        return current_pose;
    }

    inline void SetRunAwayPose(const geometry_msgs::Pose& away_pose){
        run_away_pose = away_pose;
        current_distance = CalculateDistance(current_pose, run_away_pose);
    }

    // Try to run away from the run_away_pose
    int StepForward(const int connected_num = 4){
        int step_size = 8/connected_num;
        int action_id = -1;
        for (int i = 0; i < 8; i += step_size){
            geometry_msgs::Pose attempt_pose;
            attempt_pose.position.x = current_pose.position.x + actions_x[i];
            attempt_pose.position.y = current_pose.position.y + actions_y[i];
            if (IsOccupied(*envmap, attempt_pose)){
                continue;
            }
            double attempt_distance = CalculateDistance(attempt_pose, run_away_pose);
            if (attempt_distance >= current_distance){
                action_id = i;
                current_distance = attempt_distance;
            }
            else if (no_movement_count > 5){
                // If the robot has waited for more than 5 sample periods, take a valid action
                action_id = i;
                no_movement_count--;
            }
        }
        if (action_id > -1){
            current_pose.position.x += actions_x[action_id];
            current_pose.position.y += actions_y[action_id];
        }
        else{
            no_movement_count++;
        }
        
        return action_id;
    }

    bool StepForward(const int action_id, const int connected_num){
        int step_size = 8/connected_num;
        geometry_msgs::Pose attempt_pose;
        attempt_pose.position.x = current_pose.position.x + actions_x[action_id];
        attempt_pose.position.y = current_pose.position.y + actions_y[action_id];
        if (IsOccupied(*envmap, attempt_pose)){
            return false;
        }
        current_pose.position.x += actions_x[action_id];
        current_pose.position.y += actions_y[action_id];
        return true;
    }

};

#endif