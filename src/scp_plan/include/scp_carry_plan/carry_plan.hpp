#ifndef CARRY_PLAN_HPP_
#define CARRY_PLAN_HPP_

#include <rclcpp/rclcpp.hpp>

#include <thread>
#include <future>

#include "hybrid_astar.hpp"
#include "scp_message/msg/scp_carry_task.hpp"
#include "scp_message/msg/agent_action_list.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

#define MAX_THREAD_NUM (4)

namespace scp
{

class CarryPlan
{
public:
    CarryPlan();
public:
    double v = 0, w = 0, dt = 0;
    GridMap grid_map;
    std::vector<Element> dynamic_elements;
    std::vector<Element> static_elements;
    std::vector<Element> obstacles;
    Element agent;

    scp_message::msg::ScpCarryTask task;
    CarryPlanResult plan_result;
public:
    void config(double v, double w, double dt);
    void updateElement(std::vector<Element>& dynamic_elements, std::vector<Element>& static_elements, Element& agent);
    void plan(GridMap& grid_map, scp_message::msg::ScpCarryTask& task);
    void toMsg(scp_message::msg::AgentActionList& action_list_msg);
    void toMsg(nav_msgs::msg::Path& path_msg);
private:
    void planThread(GridMap& grid_map, Pose2D& start, Pose2D& sub, Pose2D& goal, CarryPlanResult& rst);
    bool checkCollision(Element& e);
};

} // namespace scp

#endif  // CARRY_PLAN_HPP_