#include "layer1_node.hpp"
#include "tf2/utils.h"
Layer1Node::Layer1Node()
    : Node("layer1_node")
{
    plan.setExecuteSpace(0.3, M_PI / 4, 2, 2, 0.5);
    world_client = create_client<uvs_message::srv::UvQueryWorld>("uve_query_world");
}

void Layer1Node::init()
{
    graph = std::make_shared<Layer1GridGraph>(world, cv::Point3d(0.05, 0.05, M_PI / 8));
}

void Layer1Node::start_all()
{
    dynamic_sub = create_subscription<uve_message::msg::UveDynamicStatusList>("uve_dynamic_status", 1, std::bind(&Layer1Node::dynamicCallback, this, std::placeholders::_1));
    start_sub = create_subscription<geometry_msgs::msg::Pose2D>("uve_agent_status", 1, std::bind(&Layer1Node::startCallback, this, std::placeholders::_1));
    goal_sub = create_subscription<geometry_msgs::msg::PoseStamped>("inter_move_goal", 1, std::bind(&Layer1Node::goalCallback, this, std::placeholders::_1));
    path_pub = create_publisher<geometry_msgs::msg::PoseArray>("path", 1);
}

void Layer1Node::dynamicCallback(const uve_message::msg::UveDynamicStatusList::SharedPtr msg)
{
    this->dynamic = *msg;
}

void Layer1Node::startCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
{
    this->start = *msg;
}

void Layer1Node::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    this->goal.x = msg->pose.position.x;
    this->goal.y = msg->pose.position.y;
    this->goal.theta = tf2::getYaw(msg->pose.orientation);

    auto start = (*graph)(this->start.x, this->start.y, this->start.theta);
    auto goal = (*graph)(this->goal.x, this->goal.y, this->goal.theta);
    if (start == nullptr || goal == nullptr)
    {
        RCLCPP_ERROR(get_logger(), "start or goal is out of the world");
        return;
    }

    graph->updateDynamic(dynamic);
    plan.bindGraph(*graph);
    auto result = plan.search(start, goal);

    path.header.stamp = now();
    path.header.frame_id = "map";
    for (auto& point : result.path)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = point.x;
        pose.position.y = point.y;
        pose.position.z = 0;
        pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), point.z));
        path.poses.push_back(pose);
    }
    path_pub->publish(path);
}
