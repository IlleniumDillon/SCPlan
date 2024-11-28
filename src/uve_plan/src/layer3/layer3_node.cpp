#include "layer3_node.hpp"

#include "tf2/utils.h"

Layer3Node::Layer3Node()    : Node("layer3_node")
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
    declare_parameter("checkpoints");

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
    std::vector<double> checkpoints = get_parameter("checkpoints").as_double_array();

    freeGraph.load(free_graph_path);
    carryGraph.load(carry_graph_path);
    // plan2.setMaxThread(max_thread);
    // plan2.setInitGraph(freeGraph, carryGraph);
    // plan2.setFreeExecuteSpace(free_max_v, free_max_w, free_step_v, free_step_w, free_dt);
    // plan2.setCarryExecuteSpace(carry_max_v, carry_max_w, carry_step_v, carry_step_w, carry_dt);
    plan3.setMaxThread(max_thread);

    plan2_list = new Layer2Plan[max_thread];
    for (int i = 0; i < max_thread; i++)
    {
        plan2_list[i].setMaxThread(max_thread);
        plan2_list[i].setInitGraph(freeGraph, carryGraph);
        plan2_list[i].setFreeExecuteSpace(free_max_v, free_max_w, free_step_v, free_step_w, free_dt);
        plan2_list[i].setCarryExecuteSpace(carry_max_v, carry_max_w, carry_step_v, carry_step_w, carry_dt);
    }

    checkPoints.resize(checkpoints.size() / 3);
    for (int i = 0; i < checkPoints.size(); i++)
    {
        checkPoints[i] = cv::Point3d(checkpoints[i * 3], checkpoints[i * 3 + 1], checkpoints[i * 3 + 2]);
    }

    world_client = create_client<uvs_message::srv::UvQueryWorld>("uve_query_world");
    dynamic_sub = create_subscription<uve_message::msg::UveDynamicStatusList>("uve_dynamic_status", 10, std::bind(&Layer3Node::dynamicCallback, this, std::placeholders::_1));
    start_sub = create_subscription<geometry_msgs::msg::Pose2D>("uve_agent_status", 10, std::bind(&Layer3Node::startCallback, this, std::placeholders::_1));
    goal_sub = create_subscription<geometry_msgs::msg::PoseStamped>("inter_move_goal", 10, std::bind(&Layer3Node::goalCallback, this, std::placeholders::_1));
    path_pub = create_publisher<nav_msgs::msg::Path>("path", 10);
    map_pub = create_publisher<nav_msgs::msg::OccupancyGrid>("mmap", 10);
}

Layer3Node::~Layer3Node()
{
    if (plan2_list != nullptr)
    {
        delete[] plan2_list;
    }
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
    freeGraph.updateDynamic(dynamic);
    carryGraph.updateDynamic(dynamic);
    for (int i = 0; i < plan3.max_thread; i++)
    {
        plan2_list[i].updateGraph(dynamic);
    }

    map.fromGridGraph(freeGraph, dynamic, world);
    graph.fromPixMap(map, checkPoints);

    plan3.bindGraph(&graph);
    plan3.bindPixMap(&map);
    plan3.bindPlanner(plan2_list);

    cv::Point3d goal(msg->pose.position.x, msg->pose.position.y, tf2::getYaw(msg->pose.orientation));
    auto result = plan3.search(cv::Point3d(start.x, start.y, start.theta), goal);

    RCLCPP_INFO(get_logger(), "search result: %d, time: %f", result.success, result.planTime / 10.0e9);

    if (result.success)
    {
        path.poses.clear();
        path.header.stamp = now();
        path.header.frame_id = "map";
        for (auto &trace : result.path)
        {
            for(auto &point : trace.path_m)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = point.x;
                pose.pose.position.y = point.y;
                pose.pose.position.z = 0;
                pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), point.z));
                path.poses.push_back(pose);
            }
            for(auto &point : trace.path_c)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = point.x;
                pose.pose.position.y = point.y;
                pose.pose.position.z = 0;
                pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), point.z));
                path.poses.push_back(pose);
            }
            for(auto &point : trace.path_a)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = point.x;
                pose.pose.position.y = point.y;
                pose.pose.position.z = 0;
                pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), point.z));
                path.poses.push_back(pose);
            }
        }
        path_pub->publish(path);
    }
}
