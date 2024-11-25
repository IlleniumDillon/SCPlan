#ifndef LAYER2_NODE_HPP
#define LAYER2_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "layer2/layer2_plan.hpp"

#include "uvs_message/srv/uv_query_world.hpp"
#include "uve_message/msg/uve_dynamic_status_list.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "uve_message/msg/non_interactive_carry_goal.hpp"

using namespace layer2;
using namespace layer1;

class Layer2Node : public rclcpp::Node
{
public:
    Layer2Node();
    ~Layer2Node() = default;
public:
    void dynamicCallback(const uve_message::msg::UveDynamicStatusList::SharedPtr msg);
    void startCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
    void goalCallback(const uve_message::msg::NonInteractiveCarryGoal::SharedPtr msg);
public:
    std::shared_ptr<Layer1GridGraph> freeGraph;
    std::shared_ptr<Layer1GridGraph> carryGraph;
    Layer2Plan plan;
    nav_msgs::msg::Path path;
    geometry_msgs::msg::Pose2D start;
    uve_message::msg::NonInteractiveCarryGoal goal;
    uvs_message::srv::UvQueryWorld::Response world;
    uve_message::msg::UveDynamicStatusList dynamic;

    rclcpp::Client<uvs_message::srv::UvQueryWorld>::SharedPtr world_client;
    rclcpp::Subscription<uve_message::msg::UveDynamicStatusList>::SharedPtr dynamic_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr start_sub;
    rclcpp::Subscription<uve_message::msg::NonInteractiveCarryGoal>::SharedPtr goal_sub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub;

};

#endif // LAYER2_NODE_HPP