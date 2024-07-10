#ifndef HYBRID_ASTAR_HPP
#define HYBRID_ASTAR_HPP

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <tf2/utils.h>
#include "grid_map.hpp"

namespace scp
{

class HybridAStar 
{
public:
    HybridAStar();
public:
    double v = 0, w = 0, dt = 0;
    std::vector<std::pair<double, double>> control_list;
    std::vector<std::pair<double, double>> final_control_list;

    GridMap grid_map;
    std::vector<std::vector<std::pair<int, int>>> agent_occupied;
    Pose2D start_pose;
    Pose2D goal_pose;

    std::vector<Element> obstacles;
    Element agent;

    PlanResult plan_result;

    std::multimap<GridCost, GridNode*> open_set;
    std::vector<GridNode*> close_set;

    int search_mode = 0;
public:
    void config(double v, double w, double dt, int v_step, int w_step);
    void updateElement(std::vector<Element>& obstacles, Element& agent);
    void plan(GridMap& grid_map, Pose2D& start_pose, Pose2D& goal_pose, Element& agent);
    void plan(Pose2D& start_pose, Pose2D& goal_pose);
    void toMsg(nav_msgs::msg::Path& path_msg);
    void clear();
private:
    void getNeighbors(GridNode* current_node, std::vector<GridNode*>& neighbors, std::vector<GridCost>& costs, std::vector<Pose2D>& poses);
    GridCost heuristic(GridNode* node, GridNode* goal_node);
    bool checkCollision(Pose2D& pose);
    bool checkCollision(GridState& index, Pose2D& pose);
    void kinematic(Pose2D& pose0, Pose2D& pose1, double v, double w, double dt);

    // bool finalPath(GridNode* current_node, GridNode* goal_node);
};

} // namespace scp

#endif // HYBRID_ASTAR_HPP