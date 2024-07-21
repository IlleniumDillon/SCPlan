#ifndef SCP_PLAN_HPP
#define SCP_PLAN_HPP

#include <rclcpp/rclcpp.hpp>

#include "connect_graph.hpp"
#include "carry_plan.hpp"
#include "hybrid_astar.hpp"

namespace scp
{
class SCPPlan
{
public:
    SCPPlan();
public:
    double v = 0, w = 0, dt = 0;

    GridMap grid_map;
    ConnectGraph connect_graph;
    // PixMap pix_map;

    std::vector<Element> dynamic_elements;
    std::vector<Element> static_elements;
    std::vector<Element> obstacles;
    Element agent;

    CarryPlan carry_plan;
    HybridAStar hybrid_astar;

    Pose2D task;
    CarryPlanResult plan_result;
public:
    void config(double v, double w, double dt);
    void updateElement(std::vector<Element>& dynamic_elements, std::vector<Element>& static_elements, Element& agent);
    void plan(GridMap& grid_map, Pose2D& task);
    void toMsg(nav_msgs::msg::Path& path_msg);
};
} // namespace scp

#endif // SCP_PLAN_HPP