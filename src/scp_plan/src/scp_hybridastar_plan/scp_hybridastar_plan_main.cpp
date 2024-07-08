#include <rclcpp/rclcpp.hpp>
#include "scp_hybridastar_plan_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SCPHAPlanNode>());
    rclcpp::shutdown();
    return 0;
}