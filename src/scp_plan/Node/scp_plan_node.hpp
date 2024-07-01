#ifndef SCP_PLAN_NODE_HPP_
#define SCP_PLAN_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "scp_message/msg/agent_action.hpp"
#include "scp_message/msg/agent_action_list.hpp"
#include "scp_message/msg/model_state.hpp"
#include "scp_message/msg/model_state_list.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "json/json.h"

#include "Map.hpp"

class SCPPlanNode : public rclcpp::Node
{
public:
    SCPPlanNode();

public:
    std::string modelPath = "";

    scp_message::msg::ModelStateList modelStateList;
    scp_message::msg::AgentActionList agentActionList;
    nav_msgs::msg::OccupancyGrid occupancyGrid;

    rclcpp::Subscription<scp_message::msg::ModelStateList>::SharedPtr modelStateListSubscription;
    rclcpp::Publisher<scp_message::msg::AgentActionList>::SharedPtr agentActionListPublisher;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancyGridPublisher;

    ElementMap elementMap;
};

#endif // SCP_PLAN_NODE_HPP_