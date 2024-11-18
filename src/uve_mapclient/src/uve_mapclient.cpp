#include "uve_mapclient.hpp"

#include "tf2/utils.h"

UveMapClient::UveMapClient()
    : Node("uve_mapclient")
{
    sub_pose_list_ = create_subscription<uvs_message::msg::UvOptPoseList>("uvs_pose_list", 1, std::bind(&UveMapClient::pose_list_callback, this, std::placeholders::_1));
    pub_status_ = create_publisher<geometry_msgs::msg::Pose2D>("uve_agent_status", 1);
    timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&UveMapClient::timer_callback, this));
}

UveMapClient::~UveMapClient()
{
}

void UveMapClient::pose_list_callback(const uvs_message::msg::UvOptPoseList::SharedPtr msg)
{
    pose_list_ = *msg;
    auto it = std::find_if(pose_list_.pose_list.begin(), pose_list_.pose_list.end(), [](uvs_message::msg::UvOptPose pose) { return pose.name == "uv05"; });
    if (it != pose_list_.pose_list.end())
    {
        status_.x = it->pose.position.x;
        status_.y = it->pose.position.y;
        status_.theta = tf2::getYaw(it->pose.orientation);
    }
}

void UveMapClient::timer_callback()
{
    pub_status_->publish(status_);
}
