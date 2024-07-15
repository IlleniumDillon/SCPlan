#include <rclcpp/rclcpp.hpp>
#include "scp_carry_plan_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SCPCarryPlanNode>());
    rclcpp::shutdown();
    return 0;
}