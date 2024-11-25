#include "layer1_node.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

#define CONDIF_SOURCE_DIR "/home/jxl3028/Desktop/SCPlan_Exp/src/uve_plan/config"
#define CONFIG_INSTALL_DIR "/home/jxl3028/Desktop/SCPlan_Exp/install/uve_plan/share/uve_plan/config"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Layer1Node>();

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

        // node->init();
        // node->graph = std::make_shared<Layer1GridGraph>(std::string(CONDIF_SOURCE_DIR)+"/free.json");
        node->start_all();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "INIT DONE");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "WORLD FAILED");
    }

    rclcpp::spin(node);

    // node->graph->save(std::string(CONDIF_SOURCE_DIR)+"/free.json");
    // node->graph->save(std::string(CONDIF_SOURCE_DIR)+"/free.json");

    rclcpp::shutdown();
    return 0;
}