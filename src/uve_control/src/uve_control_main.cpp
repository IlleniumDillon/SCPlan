#include "rclcpp/rclcpp.hpp"
#include "uve_control/uve_control.hpp"

int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UveControl>());
    rclcpp::shutdown();
    return 0;
}