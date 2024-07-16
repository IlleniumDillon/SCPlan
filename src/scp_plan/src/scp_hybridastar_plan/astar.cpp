#include "astar.hpp"

using namespace scp;

AStarImpl::AStarImpl(GridMap &map, std::multimap<GridCost, GridNode *> &open_set, std::vector<GridNode *> &close_set, std::function<bool(Pose2D &)> checkCollision)
    : grid_map(map), open_set(open_set), close_set(close_set), checkCollision(checkCollision)
{
}

AStarImpl::~AStarImpl()
{
}

void AStarImpl::plan(Pose2D &start_pose, Pose2D &goal_pose)
{
    plan_result = PlanResult();

    auto start_index = grid_map(start_pose);
    if (start_index.node == nullptr)
    {
        RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "start pose out of map");
        return;
    }
    auto goal_index = grid_map(goal_pose);
    if (goal_index.node == nullptr)
    {
        RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "goal pose out of map");
        RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "%f, %f", goal_pose.x, goal_pose.y);
        return;
    }
    GridNode* start_node = new GridNode(start_index.node->index, start_pose);
    GridNode* goal_node = new GridNode(goal_index.node->index, goal_pose);

    start_node->search_info.g = GridCost();
    start_node->search_info.h = heuristic(start_node, goal_node);
    start_node->search_info.f = start_node->search_info.g + start_node->search_info.h;
    start_node->search_info.state = IN_OPENSET;
    start_node->search_info.it = open_set.insert(std::make_pair(start_node->search_info.f, start_node));

    

    std::vector<GridNode *> neighbors;
    std::vector<GridCost> costs;

    while (!open_set.empty())
    {
        
        plan_result.iterations++;
        auto current_node = open_set.begin()->second;
    
        if (current_node->index == goal_node->index)
        {
            GridNode *node = current_node;
            plan_result.cost = current_node->search_info.g.distance;
            while (node != nullptr)
            {
                plan_result.path.push_back(node->pose);
                node = node->search_info.parent;
            }
            std::reverse(plan_result.path.begin(), plan_result.path.end());
            plan_result.path.push_back(goal_node->pose);
            plan_result.success = true;           
            break;
        }
        
        open_set.erase(open_set.begin());
        close_set.push_back(current_node);
        current_node->search_info.state = IN_CLOSESET;
        

        getNeighbors(current_node, neighbors, costs);
        RCLCPP_INFO(rclcpp::get_logger("astar"), "neighbors: %d", neighbors.size());
        for (int i = 0; i < neighbors.size(); i++)
        {
            auto neighbor = neighbors[i];
            auto cost = costs[i];
            if (neighbor->search_info.state == IN_OPENSET)
            {
                if (neighbor->search_info.g > current_node->search_info.g + cost)
                {
                    neighbor->search_info.g = current_node->search_info.g + cost;
                    neighbor->search_info.f = neighbor->search_info.g + neighbor->search_info.h;
                    neighbor->search_info.parent = current_node;
                    open_set.erase(neighbor->search_info.it);
                    neighbor->search_info.it = open_set.insert(std::make_pair(neighbor->search_info.f, neighbor));
                }
            }
            else
            {
                neighbor->search_info.g = current_node->search_info.g + cost;
                neighbor->search_info.h = heuristic(neighbor, goal_node);
                neighbor->search_info.f = neighbor->search_info.g + neighbor->search_info.h;
                neighbor->search_info.parent = current_node;
                neighbor->search_info.state = IN_OPENSET;
                neighbor->search_info.it = open_set.insert(std::make_pair(neighbor->search_info.f, neighbor));
            }
        }
    }

    delete start_node;
    delete goal_node;
}

void AStarImpl::getNeighbors(GridNode *current_node, std::vector<GridNode *> &neighbors, std::vector<GridCost> &costs)
{
    // neighbors.clear();
    // costs.clear();

    // GridState cur = grid_map[current_node->pose];
    // for (int i = -1; i <= 1; i++)
    // {
    //     for (int j = -1; j <= 1; j++)
    //     {
    //         if (i == 0 && j == 0)
    //         {
    //             continue;
    //         }
    //         int x = cur.node->index.x + i;
    //         int y = cur.node->index.y + j;
    //         GridState next = grid_map(x, y, cur.node->index.z);
    //         if (next.node == nullptr)
    //         {
    //             continue;
    //         }   
    //         if (next.node->search_info.state == GridNodeState::IN_CLOSESET)
    //         {
    //             continue;
    //         }
    //         if (checkCollision(next.node->pose))
    //         {
    //             continue;
    //         }
    //         neighbors.push_back(next.node);
    //         GridCost cost = heuristic(current_node, next.node);
    //         costs.push_back(cost);
    //     }
    // }
}

GridCost AStarImpl::heuristic(GridNode *node, GridNode *goal_node)
{
    GridCost cost;
    cost.distance = 1. * std::sqrt(
        std::pow(node->pose.x - goal_node->pose.x, 2) + std::pow(node->pose.y - goal_node->pose.y, 2)
    );
    cost.rotation = std::abs(goal_node->pose.theta - node->pose.theta);
    if (cost.rotation >= 2 * M_PI)
    {
        cost.rotation -= 2 * M_PI;
    }
    return cost;
}
