#include "scp_plan.hpp"
#include <chrono>

using namespace scp;

SCPPlan::SCPPlan()
{
}

void SCPPlan::config(double v, double w, double dt)
{
    this->v = v;
    this->w = w;
    this->dt = dt;
}

void SCPPlan::updateElement(std::vector<Element> &dynamic_elements, std::vector<Element> &static_elements, Element &agent)
{
    this->dynamic_elements = dynamic_elements;
    this->static_elements = static_elements;
    this->agent = agent;
    obstacles.clear();
    obstacles.insert(obstacles.end(), static_elements.begin(), static_elements.end());
    obstacles.insert(obstacles.end(), dynamic_elements.begin(), dynamic_elements.end());
}

void SCPPlan::plan(GridMap &grid_map, Pose2D &task)
{
    auto start = std::chrono::steady_clock::now();
    plan_result = CarryPlanResult();
    this->grid_map = grid_map;
    this->task = task;
    this->connect_graph.convertGridMap(grid_map);
    int* pstart_domain = this->connect_graph.pix_map(agent.pose.x, agent.pose.y);
    int* pgoal_domain = this->connect_graph.pix_map(task.x, task.y);
    if (pstart_domain == nullptr || pgoal_domain == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("scp_plan"), "Start or goal domain is nullptr.");
        return;
    }
    int start_domain = *pstart_domain;
    int goal_domain = *pgoal_domain;
    if (start_domain == goal_domain)
    {
        hybrid_astar.config(v, w, dt, 2, 3);
        hybrid_astar.updateElement(obstacles, agent);
        hybrid_astar.plan(grid_map, agent.pose, task, agent);
        plan_result.cost = hybrid_astar.plan_result.cost;
        plan_result.iterations = hybrid_astar.plan_result.iterations;
        plan_result.success = hybrid_astar.plan_result.success;
        for (auto pose : hybrid_astar.plan_result.path)
        {
            plan_result.path.push_back(std::make_pair(0, pose));
        }
    }
    else
    {

    }

    auto end = std::chrono::steady_clock::now();
    plan_result.planTime = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000000.0;
}

void SCPPlan::toMsg(nav_msgs::msg::Path &path_msg)
{
    path_msg.header.stamp = rclcpp::Clock().now();
    path_msg.header.frame_id = "map";
    path_msg.poses.clear();
    for (auto pose : plan_result.path)
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.pose.position.x = pose.second.x;
        pose_msg.pose.position.y = pose.second.y;
        pose_msg.pose.position.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, pose.second.theta);
        pose_msg.pose.orientation = tf2::toMsg(q);
        path_msg.poses.push_back(pose_msg);
        // RCLCPP_INFO(rclcpp::get_logger("carry_plan"), "Path pose: (%f, %f, %f).", pose.second.x, pose.second.y, pose.second.theta);
    }
}
