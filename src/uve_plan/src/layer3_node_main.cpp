#include <rclcpp/rclcpp.hpp>

#include "layer3/layer3_node.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Layer3Node>();
    auto request = std::make_shared<uvs_message::srv::UvQueryWorld::Request>();
    while(!node->world_client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Interrupted");
            return 0; 
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service available");

    auto result = node->world_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result.get();
        node->world = *response;
        for (int i = 0; i < node->plan3.max_thread; i++)
        {
            node->plan2_list[i].setWorldDSCP(node->world);
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "INIT DONE");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "WORLD FAILED");
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}