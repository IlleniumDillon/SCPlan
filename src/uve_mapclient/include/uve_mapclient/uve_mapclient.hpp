#ifndef UVE_MAPCLIENT_HPP
#define UVE_MAPCLIENT_HPP

#include <rclcpp/rclcpp.hpp>

#include "uvs_message/srv/uv_query_world.hpp"
#include "uvs_message/msg/uv_opt_pose_list.hpp"
#include "uve_message/msg/uve_dynamic_status_list.hpp"

class UveMapClient : public rclcpp::Node
{
public:
    UveMapClient();
    ~UveMapClient();
    void init();
    void start_all();
private:
    void pose_list_callback(const uvs_message::msg::UvOptPoseList::SharedPtr msg);
    void timer_10hz_callback();
    void timer_50hz_callback();
    void cur_world_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                            const std::shared_ptr<uvs_message::srv::UvQueryWorld::Request> request,
                            std::shared_ptr<uvs_message::srv::UvQueryWorld::Response> response);
public:
    rclcpp::Client<uvs_message::srv::UvQueryWorld>::SharedPtr cli_world_dscp_;
    rclcpp::Subscription<uvs_message::msg::UvOptPoseList>::SharedPtr sub_pose_list_;
    rclcpp::Service<uvs_message::srv::UvQueryWorld>::SharedPtr ser_cur_world_;

    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_agent_status_;
    rclcpp::Publisher<uve_message::msg::UveDynamicStatusList>::SharedPtr pub_dynamic_status_;

    rclcpp::CallbackGroup::SharedPtr timer_10hz_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_50hz_callback_group_;
    rclcpp::TimerBase::SharedPtr timer_10hz_;
    rclcpp::TimerBase::SharedPtr timer_50hz_;

    uvs_message::srv::UvQueryWorld::Response cur_world_;
    uve_message::msg::UveDynamicStatusList dynamic_status_;
    geometry_msgs::msg::Pose2D agent_status_;
    uvs_message::msg::UvOptPoseList pose_list_;
    std::map<std::string, int> pose_map_;

    bool initDone = false;
};

#endif // UVE_MAPCLIENT_HPP