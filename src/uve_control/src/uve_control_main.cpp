#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rclcpp::Node>("uve_control"));
    rclcpp::shutdown();
    return 0;
}