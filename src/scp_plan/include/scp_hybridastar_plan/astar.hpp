#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <tf2/utils.h>
#include "grid_map.hpp"

namespace scp
{
class AStarImpl
{
public:
    PlanResult plan_result;
    GridMap& grid_map;
    std::multimap<GridCost, GridNode*>& open_set;
    std::vector<GridNode*>& close_set;
    std::function<bool(Pose2D& )> checkCollision;
public:
    AStarImpl(GridMap& map, std::multimap<GridCost, GridNode*>& open_set, std::vector<GridNode*>& close_set, std::function<bool(Pose2D& )> checkCollision);
    ~AStarImpl();
    void plan(Pose2D& start_pose, Pose2D& goal_pose);
private:
    void getNeighbors(GridNode* current_node, std::vector<GridNode*>& neighbors, std::vector<GridCost>& costs);
    GridCost heuristic(GridNode* node, GridNode* goal_node);
};
} // namespace scp

#endif // ASTAR_HPP