#ifndef CATCH_POINT_ROBOT_PLANNER
#define CATCH_POINT_ROBOT_PLANNER

#include <vector>
#include <limits>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>

namespace point_robot_planner{

// is in the open list, has been expended, is not touched, is being expended
enum STATUS {OPEN, CLOSED, UNTOCHED, EXPENDING};

// in current version, one step forward has to land in one of the eight connected grids
// that makes a full node which contains pose unnecessary. 
struct SimpleNode{
    int index_x;
    int index_y;
    // in-class initializer
    SimpleNode* parent = nullptr;
    int parent_action_id = 0;
    double g_cost = std::numeric_limits<double>::max()/2 - 1; // so that g_cost + h_cost won't overflow
    double h_cost = g_cost;
    double gh_cost = g_cost + h_cost;
    STATUS node_status = UNTOCHED; 

    SimpleNode& operator=(const SimpleNode& rhs){
        // I think this is the same as default =, just in case...
        this->index_x = rhs.index_x;
        this->index_y = rhs.index_y;
        this->g_cost = rhs.g_cost;
        this->h_cost = rhs.h_cost;
        this->gh_cost = rhs.gh_cost;
        // the pointer is copied, not the content it points to
        this->parent = rhs.parent;
        this->node_status = rhs.node_status;

        return *this;
    }
};

// The A* planner
class AStarPlanner {
    // vertices holds the object of nodes, other handles like start_pose_ will point to an address here
    std::vector<std::vector<SimpleNode>> vertices;
    // map of the environment, 0 is free, 100 is occupied
    nav_msgs::OccupancyGrid envmap_;

    geometry_msgs::Pose start_pose_;
    geometry_msgs::Pose goal_pose_;
    SimpleNode* start_node_;
    SimpleNode* goal_node_;

    std::vector<SimpleNode*> open_list_;

    // the resulting plan, the last element is the start_pose
    nav_msgs::Path plan_; 
    std::vector<int> path_actions_;

    // 8-connected gird, start from upper left corner, clockwise.
    const int NUM_CONNECT = 8;
    const int actions_x[8] = {-1, 0, 1, 1, 1, 0, -1, -1};
    const int actions_y[8] = {1, 1, 1, 0, -1, -1, -1, 0};
    const double actions_cost[8] {1.414, 1, 1.414, 1, 1.414, 1, 1.414, 1};

    bool is_verbose;

    inline void LocateNodeInMap(const geometry_msgs::Pose& pose_to_locate, int* index_x, int* index_y){
        *index_x = static_cast<int>(pose_to_locate.position.x/envmap_.info.resolution);
        *index_y = static_cast<int>(pose_to_locate.position.y/envmap_.info.resolution);
    }

    inline bool _IsOccupied(const SimpleNode* node_to_test){
        int id_in_map = node_to_test->index_y*envmap_.info.width + node_to_test->index_x;
        return (envmap_.data[id_in_map] != 0);
    }

    // should be replaced by real x, y if possible actions are different
    inline void CalculateHeuristicCost(SimpleNode* node_to_cal){
        double hc_sq = std::pow(node_to_cal->index_x - goal_node_->index_x, 2);
        hc_sq += std::pow(node_to_cal->index_y - goal_node_->index_y, 2);
        node_to_cal->h_cost = std::sqrt(hc_sq);
    }

    inline SimpleNode* FindNodeInGraph(const int x, const int y){
        return &(vertices[y][x]);
    }

    void NodeToPose(const SimpleNode* node, geometry_msgs::Pose& pose){
        // should be modified if the possible action changed.
        pose.position.x = node->index_x;
        pose.position.y = node->index_y;
    }

    void InitNodes();

public:
    int max_iter = 1e5;
    AStarPlanner() = default;
    AStarPlanner(const nav_msgs::OccupancyGrid& envmap, bool verbose_mode = false);
    AStarPlanner(const nav_msgs::OccupancyGrid& envmap, 
                 const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& goal_pose,
                 bool verbose_mode = false);
    void RefreshNodes();
    bool MakePlan();
    void GetPath();
    void GetPath(SimpleNode* end_node);

    inline void SetMaxIter(const int max_i){
        max_iter = max_i;
    }
    
    const nav_msgs::Path* GetPlan(){
        // it is not allowed to used the returned pointer to change plan_
        return &plan_;
    }

    const std::vector<int>* GetPlanAction(){
        return &path_actions_;
    }

    inline void SetStartPose(const geometry_msgs::Pose& start_pose){
        start_pose_ = start_pose;
        int x, y;
        LocateNodeInMap(start_pose_, &x, &y);
        start_node_ = FindNodeInGraph(x, y);
    }

    inline void SetGoalPose(const geometry_msgs::Pose& goal_pose){
        goal_pose_ = goal_pose;
        int x, y;
        LocateNodeInMap(goal_pose_, &x, &y);
        goal_node_ = FindNodeInGraph(x, y);
    }
};

}
#endif