#include "uve_mapclient.hpp"

#include "tf2/utils.h"

UveMapClient::UveMapClient()
    : Node("uve_mapclient")
{
    // sub_pose_list_ = create_subscription<uvs_message::msg::UvOptPoseList>("uvs_pose_list", 1, std::bind(&UveMapClient::pose_list_callback, this, std::placeholders::_1));
    // pub_status_ = create_publisher<geometry_msgs::msg::Pose2D>("uve_agent_status", 1);
    // timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&UveMapClient::timer_callback, this));

    cli_world_dscp_ = create_client<uvs_message::srv::UvQueryWorld>("uvs_query_world");
}

UveMapClient::~UveMapClient()
{
}

void UveMapClient::init()
{
    RCLCPP_INFO(get_logger(), "init");
    RCLCPP_INFO(get_logger(), "cargos size %d", cur_world_.cargos.size());
    for (int i = 0; i < cur_world_.cargos.size(); i++)
    {
        RCLCPP_INFO(get_logger(), "cargo %s", cur_world_.cargos[i].name.c_str());
        pose_map_[cur_world_.cargos[i].name] = i;
    }
    pose_map_[cur_world_.agents[0].name] = -1;
    RCLCPP_INFO(get_logger(), "agent %s", cur_world_.agents[0].name.c_str());
    sub_pose_list_ = create_subscription<uvs_message::msg::UvOptPoseList>("uvs_pose_list", 1, std::bind(&UveMapClient::pose_list_callback, this, std::placeholders::_1));
    initDone = false;
}

void UveMapClient::start_all()
{
    ser_cur_world_ = create_service<uvs_message::srv::UvQueryWorld>("uve_query_world", std::bind(&UveMapClient::cur_world_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    pub_agent_status_ = create_publisher<geometry_msgs::msg::Pose2D>("uve_agent_status", 1);
    pub_dynamic_status_ = create_publisher<uve_message::msg::UveDynamicStatusList>("uve_dynamic_status", 1);
    timer_10hz_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_50hz_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_10hz_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&UveMapClient::timer_10hz_callback, this), timer_10hz_callback_group_);
    timer_50hz_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&UveMapClient::timer_50hz_callback, this), timer_50hz_callback_group_);
}

void UveMapClient::pose_list_callback(const uvs_message::msg::UvOptPoseList::SharedPtr msg)
{
    pose_list_ = *msg;
    for (auto &item : pose_list_.pose_list)
    {
        if (pose_map_.find(item.name) != pose_map_.end())
        {
            if (pose_map_[item.name] == -1)
            {
                agent_status_.x = item.pose.position.x;
                agent_status_.y = item.pose.position.y;
                agent_status_.theta = tf2::getYaw(item.pose.orientation);
            }
            else
            {
                cur_world_.cargos[pose_map_[item.name]].pose.x = item.pose.position.x;
                cur_world_.cargos[pose_map_[item.name]].pose.y = item.pose.position.y;
                cur_world_.cargos[pose_map_[item.name]].pose.theta = tf2::getYaw(item.pose.orientation);
            }
        }
    }
    initDone = true;
}

void UveMapClient::timer_10hz_callback()
{
    dynamic_status_.list.clear();
    for (auto &cargo : cur_world_.cargos)
    {
        uve_message::msg::UveDynamicStatus status;
        status.name = cargo.name;
        status.pose = cargo.pose;
        dynamic_status_.list.push_back(status);
    }
    pub_dynamic_status_->publish(dynamic_status_);
}

void UveMapClient::timer_50hz_callback()
{
    agent_status_ = cur_world_.agents[0].pose;
    pub_agent_status_->publish(agent_status_);
}

void UveMapClient::cur_world_callback(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<uvs_message::srv::UvQueryWorld::Request> request, std::shared_ptr<uvs_message::srv::UvQueryWorld::Response> response)
{
    (void) request_header;
    (void) request;
    response = std::make_shared<uvs_message::srv::UvQueryWorld::Response>(cur_world_);
}
