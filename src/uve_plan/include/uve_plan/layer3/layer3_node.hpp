#ifndef LAYER3_NODE_HPP
#define LAYER3_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "layer3/layer3_plan.hpp"

#include "uvs_message/srv/uv_query_world.hpp"
#include "uve_message/msg/uve_dynamic_status_list.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "uve_message/msg/non_interactive_carry_goal.hpp"

using namespace layer3;
using namespace layer2;
using namespace layer1;

class Layer3Node : public rclcpp::Node
{
public:
    Layer3Node();
    ~Layer3Node() = default;

public:
    void dynamicCallback(const uve_message::msg::UveDynamicStatusList::SharedPtr msg);
    void startCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
public:
    Layer3PixMap map;
    Layer3SearchGraph graph;
    Layer3Plan plan3;
    Layer2Plan plan2;
    Layer1GridGraph freeGraph;
    Layer1GridGraph carryGraph;
    std::vector<cv::Point3d> checkPoints;

    nav_msgs::msg::Path path;
    geometry_msgs::msg::Pose2D start;
    geometry_msgs::msg::Pose2D goal;
    uvs_message::srv::UvQueryWorld::Response world;
    uve_message::msg::UveDynamicStatusList dynamic;

    rclcpp::Client<uvs_message::srv::UvQueryWorld>::SharedPtr world_client;
    rclcpp::Subscription<uve_message::msg::UveDynamicStatusList>::SharedPtr dynamic_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr start_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
};

#endif // LAYER3_NODE_HPP