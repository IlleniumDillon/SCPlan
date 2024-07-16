#ifndef SCP_CARRY_PLAN_NODE_HPP
#define SCP_CARRY_PLAN_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include "carry_plan.hpp"

using namespace scp;

class SCPCarryPlanNode : public rclcpp::Node
{
public:
    SCPCarryPlanNode();
};

#endif // SCP_CARRY_PLAN_NODE_HPP