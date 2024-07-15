#include "hybrid_astar.hpp"

using namespace scp;

HybridAStar::HybridAStar()
{
}

void HybridAStar::config(double v, double w, double dt, int v_step, int w_step)
{
    this->v = v;
    this->w = w;
    this->dt = dt;

    double v_resolution = 2 * v / v_step;
    double w_resolution = 2 * w / w_step;

    control_list.clear();
    control_list.push_back(
                std::make_pair(v, -w)
            );
    control_list.push_back(
                std::make_pair(v, 0)
            );
    control_list.push_back(
                std::make_pair(v, w)
            );

    final_control_list.clear();
    final_control_list.push_back(
                std::make_pair(v, -w)
            );
    final_control_list.push_back(
                std::make_pair(v, 0)
            );
    final_control_list.push_back(
                std::make_pair(v, w)
            );
    final_control_list.push_back(
                std::make_pair(0, -w)
            );
    final_control_list.push_back(
                std::make_pair(0, w)
            );
    final_control_list.push_back(
                std::make_pair(-v, -w)
            );
    final_control_list.push_back(
                std::make_pair(-v, 0)
            );
    final_control_list.push_back(
                std::make_pair(-v, w)
            );
    
}

void scp::HybridAStar::updateElement(std::vector<Element> &obstacles, Element &agent)
{
    this->obstacles = obstacles;
    this->agent = agent;
}

void HybridAStar::plan(GridMap &grid_map, Pose2D &start_pose, Pose2D &goal_pose, Element &agent)
{
    plan_result = PlanResult();
    this->grid_map = grid_map;
    search_mode = 0;
    auto timeStart = std::chrono::steady_clock::now();
    plan(start_pose, goal_pose);
    auto timeEnd = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(timeEnd - timeStart);
    plan_result.planTime = (float)duration.count() / 1000;
}

void HybridAStar::plan(Pose2D &start_pose, Pose2D &goal_pose)
{
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
    std::vector<Pose2D> poses;

    while (!open_set.empty())
    {
        
        plan_result.iterations++;
        auto current_node = open_set.begin()->second;
        
        // if (search_mode == 0 && current_node->search_info.h.distance < 0.5)
        // {
        //     RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "switch to final search mode");
        //     search_mode = 1;
        // }

        // if (current_node->index == goal_node->index)
        if(current_node->search_info.h.distance < 0.5)
        {
            auto curPose = current_node->pose;
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
            // clear();
            // AStarImpl astar_impl(grid_map, open_set, close_set, std::bind(&HybridAStar::checkCollision, this, std::placeholders::_1));
            // astar_impl.plan(start_pose, goal_pose);
            // if (astar_impl.plan_result.success)
            // {
            //     plan_result.cost += astar_impl.plan_result.cost;
            //     plan_result.path.insert(plan_result.path.end(), astar_impl.plan_result.path.begin(), astar_impl.plan_result.path.end());
            //     plan_result.iterations += astar_impl.plan_result.iterations;
            //     RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "final search success");
            // }
            // else
            // {
            //     RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "final search failed");
            // }
            // clear();
            break;
        }
        
        open_set.erase(open_set.begin());
        close_set.push_back(current_node);
        current_node->search_info.state = IN_CLOSESET;
        

        getNeighbors(current_node, neighbors, costs, poses);
        // RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "neighbors: %d", neighbors.size());
        for (int i = 0; i < neighbors.size(); i++)
        {
            auto neighbor = neighbors[i];
            auto cost = costs[i];
            auto pose = poses[i];
            if (neighbor->search_info.state == IN_OPENSET)
            {
                if (neighbor->search_info.g > current_node->search_info.g + cost)
                {
                    neighbor->pose = pose;
                    neighbor->search_info.g = current_node->search_info.g + cost;
                    neighbor->search_info.f = neighbor->search_info.g + neighbor->search_info.h;
                    neighbor->search_info.parent = current_node;
                    open_set.erase(neighbor->search_info.it);
                    neighbor->search_info.it = open_set.insert(std::make_pair(neighbor->search_info.f, neighbor));
                }
            }
            else
            {
                neighbor->pose = pose;
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

    clear();
}

GridCost HybridAStar::heuristic(GridNode *node, GridNode *goal_node)
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

bool HybridAStar::checkCollision(Pose2D& pose)
{
    Element temp = agent;
    temp.updatePose(pose);

    for (auto point : temp.currentVertices)
    {
        auto index = grid_map(point);
        if (index.node == nullptr)
        {
            return true;
        }
        if (index.occupied)
        {
            return true;
        }
    }

    return false;
}

// bool HybridAStar::checkCollision(GridState &index, Pose2D &pose)
// {
//     Element temp = agent;
//     temp.updatePose(pose);

//     for (auto i : *(index.collision))
//     {
//         if (temp.isCollision(obstacles[i]))
//         {
//             return true;
//         }
//     }

//     return false;
// }

void scp::HybridAStar::kinematic(Pose2D &pose0, Pose2D &pose1, double v, double w, double dt)
{
    if (pose0.theta >= 2 * M_PI)
    {
        pose0.theta -= 2 * M_PI;
    }
    if (pose0.theta < 0)
    {
        pose0.theta += 2 * M_PI;
    }
    double dtheta = w * dt;
    pose1.theta = pose0.theta + dtheta;
    if (pose1.theta >= 2 * M_PI)
    {
        pose1.theta -= 2 * M_PI;
    }
    if (pose1.theta < 0)
    {
        pose1.theta += 2 * M_PI;
    }
    if (abs(dtheta) < 1e-6)
    {
        double dx = v * dt * std::cos(pose0.theta);
        double dy = v * dt * std::sin(pose0.theta);

        pose1.x = pose0.x + dx;
        pose1.y = pose0.y + dy;
    }
    else
    {
        double R = v / w;
        double Choord = std::sqrt(
            2 * R * R * (1 - std::cos(dtheta))
        );
        double dx = Choord * std::cos(pose0.theta + dtheta / 2);
        double dy = Choord * std::sin(pose0.theta + dtheta / 2);

        pose1.x = pose0.x + dx;
        pose1.y = pose0.y + dy;
    }
}

bool HybridAStar::finalPath(GridNode *current_node, GridNode *goal_node)
{
    return false;
}

void HybridAStar::getNeighbors(GridNode *current_node, std::vector<GridNode *> &neighbors, std::vector<GridCost> &costs, std::vector<Pose2D>& poses)
{
    
    neighbors.clear();
    costs.clear();
    poses.clear();
    

    double c_x = current_node->pose.x;
    double c_y = current_node->pose.y;
    double c_theta = current_node->pose.theta;


    for (auto control : control_list)
    {
        Pose2D n_pose;
        kinematic(current_node->pose, n_pose, control.first, control.second, dt);
        auto n_index = grid_map(n_pose);
        double n_x = n_pose.x;
        double n_y = n_pose.y;
        double n_theta = n_pose.theta;
        
        if (n_index.node == nullptr)
        {
            
            continue;
        }
        
        if (n_index.node->index == current_node->index)
        {
            
            continue;
        }
        
        if (n_index.node->search_info.state == IN_CLOSESET)
        {
            
            continue;
        }
        
        if (checkCollision(n_pose))
        // if (checkCollision(n_index, n_pose))
        {
            continue;
        }
        
        neighbors.push_back(n_index.node);
        GridCost cost;
        cost.distance = std::sqrt(std::pow(n_x - c_x, 2) + std::pow(n_y - c_y, 2));
        cost.rotation = std::abs(n_theta - c_theta);
        costs.push_back(cost);
        poses.push_back(n_pose);
    }
}

void HybridAStar::clear()
{
    for (auto node : close_set)
    {
        node->search_info = GridNodeSearchInfo();
    }
    for (auto node : open_set)
    {
        node.second->search_info = GridNodeSearchInfo();
    }
    open_set.clear();
    close_set.clear();
}

void HybridAStar::toMsg(nav_msgs::msg::Path &path_msg)
{
    path_msg.header.stamp = rclcpp::Clock().now();
    path_msg.header.frame_id = "map";
    path_msg.poses.clear();
    for (auto pose : plan_result.path)
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.pose.position.x = pose.x;
        pose_msg.pose.position.y = pose.y;
        pose_msg.pose.position.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, pose.theta);
        pose_msg.pose.orientation = tf2::toMsg(q);
        path_msg.poses.push_back(pose_msg);
    }
}
