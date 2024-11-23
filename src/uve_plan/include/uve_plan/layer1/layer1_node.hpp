#ifndef LAYER1_NODE_HPP
#define LAYER1_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "layer1_plan.hpp"

#include "uvs_message/srv/uv_query_world.hpp"
#include "uve_message/msg/uve_dynamic_status_list.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace layer1;

class Layer1Node : public rclcpp::Node
{
public:
    Layer1Node();
    ~Layer1Node() = default;
    void init();
    void start_all();
public:
    void dynamicCallback(const uve_message::msg::UveDynamicStatusList::SharedPtr msg);
    void startCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
public:
    std::shared_ptr<Layer1GridGraph> graph;
    Layer1Plan plan;
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

#endif // LAYER1_NODE_HPP