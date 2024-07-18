#include <rclcpp/rclcpp.hpp>

#include "scp_plan_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScpPlanNode>());
    rclcpp::shutdown();
    return 0;
}