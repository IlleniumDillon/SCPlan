#ifndef SCP_PLAN_NODE_HPP_
#define SCP_PLAN_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2/utils.h"

#include "scp_message/msg/agent_action.hpp"
#include "scp_message/msg/agent_action_list.hpp"
#include "scp_message/msg/model_state.hpp"
#include "scp_message/msg/model_state_list.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "json/json.h"
#include "opencv4/opencv2/opencv.hpp"

#include "Map.hpp"
#include "HybridAStar.hpp"
#include "Visualize.hpp"

class SCPPlanNode : public rclcpp::Node
{
public:
    SCPPlanNode();

public:
    std::string modelPath = "";
    Element agentTemplate;
    Element goodTemplate;
    Element obstacleTemplate;
    Element wall10Template;
    Element wall12Template;

    scp_message::msg::ModelStateList modelStateList;
    scp_message::msg::AgentActionList agentActionList;
    nav_msgs::msg::OccupancyGrid occupancyGrid;
    nav_msgs::msg::Path path;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr agentTargetPoseSubscription;
    rclcpp::Subscription<scp_message::msg::ModelStateList>::SharedPtr modelStateListSubscription;
    rclcpp::Publisher<scp_message::msg::AgentActionList>::SharedPtr agentActionListPublisher;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancyGridPublisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher;

    rclcpp::TimerBase::SharedPtr timer;

    ElementMap elementMap;

    std::vector<Element> elementsShadow;
    Element agentShadow;
    double sceneWidth, sceneHeight;
    double agentV, agentW, agentDt;
    double solveResolution, distanceMapResolution;

private:
    bool loadModelTemplate(std::string modelName, Element& element);

    void agentTargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void modelStateListCallback(const scp_message::msg::ModelStateList::SharedPtr msg);
    void timerCallback();
};

#endif // SCP_PLAN_NODE_HPP_