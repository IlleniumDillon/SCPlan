#ifndef SCP_PLAN_NODE_HPP
#define SCP_PLAN_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include "carry_plan.hpp"

using namespace scp;

class ScpPlanNode : public rclcpp::Node 
{
public:
    ScpPlanNode();
};

#endif // SCP_PLAN_NODE_HPP