#include "layer3_node.hpp"

#include "tf2/utils.h"

Layer3Node::Layer3Node()
    : Node("layer3_node")
{
    declare_parameter("free_graph_path");
    declare_parameter("carry_graph_path");
    declare_parameter("max_thread");
    declare_parameter("freespace.max_v");
    declare_parameter("freespace.max_w");
    declare_parameter("freespace.step_v");
    declare_parameter("freespace.step_w");
    declare_parameter("freespace.dt");
    declare_parameter("carryspace.max_v");
    declare_parameter("carryspace.max_w");
    declare_parameter("carryspace.step_v");
    declare_parameter("carryspace.step_w");
    declare_parameter("carryspace.dt");

    std::string free_graph_path = get_parameter("free_graph_path").as_string();
    std::string carry_graph_path = get_parameter("carry_graph_path").as_string();
    int max_thread = get_parameter("max_thread").as_int();
    double free_max_v = get_parameter("freespace.max_v").as_double();
    double free_max_w = get_parameter("freespace.max_w").as_double();
    int free_step_v = get_parameter("freespace.step_v").as_int();
    int free_step_w = get_parameter("freespace.step_w").as_int();
    double free_dt = get_parameter("freespace.dt").as_double();
    double carry_max_v = get_parameter("carryspace.max_v").as_double();
    double carry_max_w = get_parameter("carryspace.max_w").as_double();
    int carry_step_v = get_parameter("carryspace.step_v").as_int();
    int carry_step_w = get_parameter("carryspace.step_w").as_int();
    double carry_dt = get_parameter("carryspace.dt").as_double();

    freeGraph.load(free_graph_path);
    carryGraph.load(carry_graph_path);
    plan.setMaxThread(max_thread);
    plan.setInitGraph(freeGraph, carryGraph);
    plan.setFreeExecuteSpace(free_max_v, free_max_w, free_step_v, free_step_w, free_dt);
    plan.setCarryExecuteSpace(carry_max_v, carry_max_w, carry_step_v, carry_step_w, carry_dt);

    world_client = create_client<uvs_message::srv::UvQueryWorld>("uve_query_world");
    dynamic_sub = create_subscription<uve_message::msg::UveDynamicStatusList>("uve_dynamic_status", 10, std::bind(&Layer3Node::dynamicCallback, this, std::placeholders::_1));
    start_sub = create_subscription<geometry_msgs::msg::Pose2D>("uve_agent_status", 10, std::bind(&Layer3Node::startCallback, this, std::placeholders::_1));
    goal_sub = create_subscription<geometry_msgs::msg::PoseStamped::SharedPtr>("inter_move_goal", 10, std::bind(&Layer3Node::goalCallback, this, std::placeholders::_1));
    path_pub = create_publisher<nav_msgs::msg::Path>("path", 10);
    map_pub = create_publisher<nav_msgs::msg::OccupancyGrid>("mmap", 10);
    pose_pub = create_publisher<geometry_msgs::msg::PoseArray>("trace", 10);
}

void Layer3Node::dynamicCallback(const uve_message::msg::UveDynamicStatusList::SharedPtr msg)
{
    dynamic = *msg;
}

void Layer3Node::startCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
{
    start = *msg;
}

void Layer3Node::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    goal = *msg;
    plan.updateGraph(dynamic);
    auto ret = plan.search(
        cv::Point3d(start.x, start.y, start.theta),
        cv::Point3d(goal.pose.position.x, goal.pose.position.y, tf2::getYaw(goal.pose.orientation))
    );
    RCLCPP_INFO(get_logger(), "search result: %d, time: %f", ret.success, ret.planTime / 10.0e9);
}
