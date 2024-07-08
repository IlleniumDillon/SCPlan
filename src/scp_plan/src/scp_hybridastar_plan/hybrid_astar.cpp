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
    // for (double v_ = -v; v_ <= v; v_ += v_resolution)
    // {
    //     for (double w_ = -w; w_ <= w; w_ += w_resolution)
    //     {
    //         control_list.push_back(
    //             std::make_pair(v_, w_)
    //         );
    //     }
    // }

    for (double v_ = -v; v_ <= v; v_ += v_resolution)
    {
        control_list.push_back(
            std::make_pair(v_, 0)
        );
    }

    for (double w_ = -w; w_ <= w; w_ += w_resolution)
    {
        control_list.push_back(
            std::make_pair(0, w_)
        );
    }

}

void HybridAStar::plan(GridMap &grid_map, Pose2D &start_pose, Pose2D &goal_pose, Element &agent)
{
    plan_result = PlanResult();
    this->grid_map = grid_map;
    Element temp = agent;
    agent_occupied.clear();
    for (int i = 0; i < grid_map.height; i++)
    {
        Pose2D pose;
        pose.x = 0;
        pose.y = 0;
        pose.theta = grid_map.yawList[i];
        temp.updatePose(pose);
        std::vector<std::pair<int, int>> occupied;
        temp.getDispersed(occupied, grid_map.positionResolution, grid_map.oriX, grid_map.oriY);
        agent_occupied.push_back(occupied);
    }
    plan(start_pose, goal_pose);
    // auto start_index = grid_map(start_pose);
    // if (start_index.node == nullptr)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "start pose out of map");
    //     return;
    // }
    // auto goal_index = grid_map(goal_pose);
    // if (goal_index.node == nullptr)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "goal pose out of map");
    //     RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "%f, %f", goal_pose.x, goal_pose.y);
    //     return;
    // }
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

    //RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "1");

    start_node->search_info.g = GridCost();
    start_node->search_info.h = heuristic(start_node, goal_node);
    start_node->search_info.f = start_node->search_info.g + start_node->search_info.h;
    start_node->search_info.state = IN_OPENSET;
    start_node->search_info.it = open_set.insert(std::make_pair(start_node->search_info.f, start_node));

    //RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "2");

    std::vector<GridNode *> neighbors;
    std::vector<GridCost> costs;
    std::vector<Pose2D> poses;

    while (!open_set.empty())
    {
        //RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "_3");
        plan_result.iterations++;
        auto current_node = open_set.begin()->second;
        //RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "_4");
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
        //RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "_5");
        open_set.erase(open_set.begin());
        close_set.push_back(current_node);
        current_node->search_info.state = IN_CLOSESET;
        //RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "_6");

        getNeighbors(current_node, neighbors, costs, poses);
        //RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "_7");
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
    cost.distance = std::sqrt(
        std::pow(node->pose.x - goal_node->pose.x, 2) + std::pow(node->pose.y - goal_node->pose.y, 2)
    );
    cost.rotation = goal_node->pose.theta - node->pose.theta;
    if (cost.rotation > M_PI)
    {
        cost.rotation -= 2 * M_PI;
    }
    else if (cost.rotation < -M_PI)
    {
        cost.rotation += 2 * M_PI;
    }
    return cost;
}

bool HybridAStar::checkCollision(Pose2D& pose)
{
    int dx = pose.x / grid_map.positionResolution;
    int dy = pose.y / grid_map.positionResolution;
    int dz = (int)(pose.theta / grid_map.yawResolution) % grid_map.depth;
    dz = dz < 0 ? dz + grid_map.depth : dz;

    for (auto occupied : agent_occupied[dz])
    {
        int x = dx + occupied.first;
        int y = dy + occupied.second;
        auto index = grid_map(x, y, dz);
        if (index.node == nullptr || index.occupied)
        {
            return true;
        }
    }
}

void HybridAStar::getNeighbors(GridNode *current_node, std::vector<GridNode *> &neighbors, std::vector<GridCost> &costs, std::vector<Pose2D>& poses)
{
    //RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "N1");
    neighbors.clear();
    costs.clear();
    poses.clear();
    //RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "N2");

    double c_x = current_node->pose.x;
    double c_y = current_node->pose.y;
    double c_theta = current_node->pose.theta;

    //RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "N3");
    for (auto control : control_list)
    {
        //RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "_N4");
        double v = control.first;
        double w = control.second;
        double n_x = c_x + v * std::cos(c_theta) * dt;
        double n_y = c_y + v * std::sin(c_theta) * dt;
        double n_theta = c_theta + w * dt;
        n_theta = std::fmod(n_theta, 2 * M_PI);
        Pose2D n_pose;
        n_pose.x = n_x;
        n_pose.y = n_y;
        n_pose.theta = n_theta;
        auto n_index = grid_map(n_pose);
        //RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "_N5");
        if (n_index.node == nullptr)
        {
            //RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "_N6");
            continue;
        }
        //RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "_N7");
        if (n_index.node->index == current_node->index)
        {
            //RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "_N8");
            continue;
        }
        //RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "_N9");
        if (n_index.node->search_info.state == IN_CLOSESET)
        {
            //RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "_N10");
            continue;
        }
        //RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "_N11");
        if (checkCollision(n_pose))
        {
            //RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "_N12");
            continue;
        }
        //RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "_N13");
        neighbors.push_back(n_index.node);
        GridCost cost;
        cost.distance = std::sqrt(std::pow(n_x - c_x, 2) + std::pow(n_y - c_y, 2));
        cost.rotation = std::abs(n_theta - c_theta);
        costs.push_back(cost);
        poses.push_back(n_pose);
        //RCLCPP_INFO(rclcpp::get_logger("scp_hybridastar_plan"), "_N14");
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
