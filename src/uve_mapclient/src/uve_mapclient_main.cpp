#include <rclcpp/rclcpp.hpp>
#include "uve_mapclient.hpp"

#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UveMapClient>();
    
    auto request = std::make_shared<uvs_message::srv::UvQueryWorld::Request>();
    while(!node->cli_world_dscp_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Interrupted");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = node->cli_world_dscp_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result.get();
        node->cur_world_ = *response;
        node->init();
        while(!node->initDone)
        {
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Interrupted");
                return 0;
            }
            rclcpp::spin_some(node);
        }
        node->start_all();
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