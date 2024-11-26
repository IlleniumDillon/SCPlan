#include "layer2_node.hpp"

#include "tf2/utils.h"

Layer2Node::Layer2Node()
    : Node("layer2_node")
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
    // RCLCPP_INFO(get_logger(), "free_graph size: %d %d %d", freeGraph->size.x, freeGraph->size.y, freeGraph->size.z);
    carryGraph.load(carry_graph_path);
    // RCLCPP_INFO(get_logger(), "carry_graph size: %d %d %d", carryGraph->size.x, carryGraph->size.y, carryGraph->size.z);
    plan.setMaxThread(max_thread);
    plan.setInitGraph(freeGraph, carryGraph);
    plan.setFreeExecuteSpace(free_max_v, free_max_w, free_step_v, free_step_w, free_dt);
    plan.setCarryExecuteSpace(carry_max_v, carry_max_w, carry_step_v, carry_step_w, carry_dt);

    world_client = create_client<uvs_message::srv::UvQueryWorld>("uve_query_world");
    dynamic_sub = create_subscription<uve_message::msg::UveDynamicStatusList>("uve_dynamic_status", 10, std::bind(&Layer2Node::dynamicCallback, this, std::placeholders::_1));
    start_sub = create_subscription<geometry_msgs::msg::Pose2D>("uve_agent_status", 10, std::bind(&Layer2Node::startCallback, this, std::placeholders::_1));
    goal_sub = create_subscription<uve_message::msg::NonInteractiveCarryGoal>("nscp_carry_goal", 10, std::bind(&Layer2Node::goalCallback, this, std::placeholders::_1));
    path_pub = create_publisher<nav_msgs::msg::Path>("path", 10);
    map_pub = create_publisher<nav_msgs::msg::OccupancyGrid>("mmap", 10);
    pose_pub = create_publisher<geometry_msgs::msg::PoseArray>("trace", 10);
}

void Layer2Node::dynamicCallback(const uve_message::msg::UveDynamicStatusList::SharedPtr msg)
{
    dynamic = *msg;
}

void Layer2Node::startCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
{
    start = *msg;
}

void Layer2Node::goalCallback(const uve_message::msg::NonInteractiveCarryGoal::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "goalCallback");
    goal = *msg;
    plan.updateGraph(dynamic);

    auto graph = &plan.carryGraphs[0];
    nav_msgs::msg::OccupancyGrid map;
    map.header.stamp = now();
    map.header.frame_id = "map";
    map.info.origin.position.x = graph->state_min.x;
    map.info.origin.position.y = graph->state_min.y;
    map.info.resolution = 0.05;
    map.info.width = graph->size.x;
    map.info.height = graph->size.y;
    map.data.resize(graph->XY);
    for (int x_ = 0; x_ < graph->size.x; x_++)
    {
        for (int y_ = 0; y_ < graph->size.y; y_++)
        {
            uint8_t value = 0;
            for (int theta_ = 0; theta_ < graph->size.z; theta_++)
            {
                if ((*graph)(x_, y_, theta_)->collision_static || (*graph)(x_, y_, theta_)->collision_dynamic)
                {
                    value += 5;
                }
            }
            map.data[x_ + y_ * graph->size.x] = value;
        }
    }
    map_pub->publish(map);

    auto result = plan.search(
        cv::Point3d(start.x, start.y, start.theta), 
        goal.name, 
        cv::Point3d(goal.goal_cargo.x, goal.goal_cargo.y, goal.goal_cargo.theta), 
        cv::Point3d(goal.goal_agent.x, goal.goal_agent.y, goal.goal_agent.theta));
    RCLCPP_INFO(get_logger(), "search result: %d, time: %f", result.success, result.planTime/10.0e9);
    if (result.success)
    {   
        geometry_msgs::msg::PoseArray poses;
        poses.header.stamp = now();
        poses.header.frame_id = "map";
        path.poses.clear();
        path.header.stamp = now();
        path.header.frame_id = "map";
        for (auto& point : result.path_m)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.position.z = 0;
            pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), point.z));
            path.poses.push_back(pose);
            geometry_msgs::msg::Pose p;
            p.position = pose.pose.position;
            p.orientation = pose.pose.orientation;
            poses.poses.push_back(p);
        }
        for (auto& point : result.path_c)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.position.z = 0;
            pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), point.z));
            path.poses.push_back(pose);
            geometry_msgs::msg::Pose p;
            p.position = pose.pose.position;
            p.orientation = pose.pose.orientation;
            poses.poses.push_back(p);
        }
        for (auto& point : result.path_a)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.position.z = 0;
            pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), point.z));
            path.poses.push_back(pose);
            geometry_msgs::msg::Pose p;
            p.position = pose.pose.position;
            p.orientation = pose.pose.orientation;
            poses.poses.push_back(p);
        }
        path_pub->publish(path);
        pose_pub->publish(poses);
    }
}
