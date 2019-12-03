#include "planners.hpp"
#include "utility.hpp"

namespace point_robot_planner{

void AStarPlanner::InitNodes(){
    for (int i = 0; i < envmap_.info.height; ++i){
        std::vector<SimpleNode> row;
        for (int j = 0; j < envmap_.info.width; ++j){
            SimpleNode temp_node;
            temp_node.index_x = j;
            temp_node.index_y = i;
            row.push_back(temp_node);
        }
        vertices.push_back(row);
    }
}

// reset g cost and h cost to maximum double
void AStarPlanner::RefreshNodes(){
    for (int i = 0; i < envmap_.info.height; ++i){
        for (int j = 0; j < envmap_.info.width; ++j){
            vertices[i][j].parent = nullptr;
            vertices[i][j].node_status = UNTOCHED;
            CalculateHeuristicCost(&vertices[i][j]);
            vertices[i][j].g_cost = std::numeric_limits<double>::max()/2 - 1;
            vertices[i][j].gh_cost = vertices[i][j].g_cost + vertices[i][j].gh_cost;
        }
    }
}

AStarPlanner::AStarPlanner(const nav_msgs::OccupancyGrid& envmap, bool verbose_mode){
    is_verbose = verbose_mode;
    plan_.header.frame_id = "map";
    envmap_ = envmap;
    InitNodes();
    geometry_msgs::Pose pose_default;
    SetGoalPose(pose_default);
    SetStartPose(pose_default);
    RefreshNodes();
}

AStarPlanner::AStarPlanner(const nav_msgs::OccupancyGrid& envmap, 
                            const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& goal_pose,
                            bool verbose_mode){
    is_verbose = verbose_mode;
    plan_.header.frame_id = "map";
    envmap_ = envmap;
    InitNodes();
    SetGoalPose(goal_pose);
    SetStartPose(start_pose);
    RefreshNodes();
}

// Recover planned path from the goal node to the start node
void AStarPlanner::GetPath(){
    plan_.poses.clear();
    SimpleNode* current_node = goal_node_;
    while(current_node->parent != nullptr){
        geometry_msgs::PoseStamped temp_pose;
        NodeToPose(current_node, temp_pose.pose);
        plan_.poses.push_back(temp_pose);
        path_actions_.push_back(current_node->parent_action_id);
        current_node = current_node->parent;
    }
    plan_.header.stamp = ros::Time().now();
}

// recover planned path from a given node to the start node
void AStarPlanner::GetPath(SimpleNode* end_node){
    plan_.poses.clear();
    SimpleNode* current_node = end_node;
    while(current_node->parent != nullptr){
        geometry_msgs::PoseStamped temp_pose;
        NodeToPose(current_node, temp_pose.pose);
        plan_.poses.push_back(temp_pose);
        path_actions_.push_back(current_node->parent_action_id);
        current_node = current_node->parent;
    }
    plan_.header.stamp = ros::Time().now();
}

// peform A* search, if successed, return true, store the results into plan_ 
bool AStarPlanner::MakePlan(){
    open_list_.clear();
    start_node_->parent = nullptr;
    start_node_->g_cost = 0;
    start_node_->node_status = OPEN;
    open_list_.push_back(start_node_);
    int iter_counter = 0;

    while (!open_list_.empty() && iter_counter < max_iter){
        ++iter_counter;

        SimpleNode* current_node = open_list_.back();
        current_node->node_status = EXPENDING;
        open_list_.pop_back();
        
        // reached goal?
        if (goal_node_->node_status == EXPENDING){
            GetPath(); 
            ROS_INFO("Iterations: %d", iter_counter);
            return true;
        }

        // expending current node
        for (int i = 0; i < NUM_CONNECT; ++i){
            int next_x = current_node->index_x + actions_x[i];
            int next_y = current_node->index_y + actions_y[i];
            if (next_x < 0 || next_y < 0 || next_y >= envmap_.info.height || next_x >= envmap_.info.width){
                // movement out of range
                continue;
            }
            SimpleNode* next_node = FindNodeInGraph(next_x, next_y);
            if (_IsOccupied(next_node)){
                // obstacles
                continue;
            }
            
            double g_cost_next = current_node->g_cost + actions_cost[i];
            // note that heuristic cost is only used for prioritizing the open_list_
            if (next_node->node_status != CLOSED){
                if (g_cost_next < next_node->g_cost){
                    next_node->g_cost = g_cost_next;
                    next_node->gh_cost = next_node->g_cost + next_node->h_cost;
                    next_node->parent = current_node;
                    next_node->parent_action_id = i;
                    if (next_node->node_status != OPEN){
                        next_node->node_status = OPEN;
                        open_list_.push_back(next_node);
                    }
                }
            }
            //if (g_cost_next < next_node->g_cost){
                //next_node->g_cost = g_cost_next;
                //next_node->gh_cost = next_node->g_cost + next_node->h_cost;
                //next_node->parent = current_node;
                //if (next_node->node_status != OPEN && next_node->node_status != CLOSED){
                    //next_node->node_status = OPEN;
                    //next_node->parent_action_id = i;
                    //open_list_.push_back(next_node);
                //}
            //}
        }
        current_node->node_status = CLOSED;

        //make sure the last element has the smallest gh_cost, linear time search
        if (!open_list_.empty()){
            double gh_min = open_list_.back()->gh_cost;
            int id_min = 0;
            for (int i = 0; i < open_list_.size(); ++i){
                if (gh_min > open_list_[i]->gh_cost){
                    id_min = i;
                    gh_min = open_list_[i]->gh_cost;
                }
            }
            SimpleNode* temp_node = open_list_[id_min];
            open_list_.erase(id_min + open_list_.begin());
            open_list_.push_back(temp_node);
        }

    }

    // search failed
    if (open_list_.empty()){
        ROS_WARN_ONCE("No plan is found. Check the connectivity between the initial pose and the goal pose");
    }
    else{
        if (is_verbose){
            ROS_WARN_ONCE("No plan found after maximum literation (%d)", max_iter);
            ROS_WARN_ONCE("Try to increase max_iter");
            ROS_INFO("The halfway path is generated");
        }
        GetPath(open_list_.back());
    }
    return false;
}


}