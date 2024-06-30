#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("scp_simulate_manual");
  RCLCPP_INFO(node->get_logger(), "Hello, SCP Simulate Manual Node!");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}