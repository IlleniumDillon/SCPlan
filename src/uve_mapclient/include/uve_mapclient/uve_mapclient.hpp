#ifndef UVE_MAPCLIENT_HPP
#define UVE_MAPCLIENT_HPP

#include <rclcpp/rclcpp.hpp>

// #include "uvs_message/srv/uv_query_element.hpp"
// #include "uvs_message/msg/uv_emb_status.hpp"
// #include "uve_message/msg/uve_agent_status.hpp"
#include "uvs_message/msg/uv_opt_pose_list.hpp"
#include "uve_message/msg/uve_agent_status.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

class UveMapClient : public rclcpp::Node
{
public:
    UveMapClient();
    ~UveMapClient();
private:
    void pose_list_callback(const uvs_message::msg::UvOptPoseList::SharedPtr msg);
    void timer_callback();
private:
    rclcpp::Subscription<uvs_message::msg::UvOptPoseList>::SharedPtr sub_pose_list_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_status_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Pose2D status_;
    uvs_message::msg::UvOptPoseList pose_list_;
};

#endif // UVE_MAPCLIENT_HPP