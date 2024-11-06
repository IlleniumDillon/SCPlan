#ifndef UVE_MAPCLIENT_HPP
#define UVE_MAPCLIENT_HPP

#include <rclcpp/rclcpp.hpp>

#include "uvs_message/srv/uv_query_element.hpp"
#include "uvs_message/msg/uv_emb_status.hpp"
#include "uve_message/msg/uve_agent_status.hpp"

class UveMapClient : public rclcpp::Node
{
public:
    UveMapClient();
    ~UveMapClient();
private:
    void uvEmbStatusCallback(const uvs_message::msg::UvEmbStatus::SharedPtr msg);
private:
    rclcpp::Subscription<uvs_message::msg::UvEmbStatus>::SharedPtr uv_emb_status_sub_;
    rclcpp::Client<uvs_message::srv::UvQueryElement>::SharedPtr uv_query_element_client_;
};

#endif // UVE_MAPCLIENT_HPP