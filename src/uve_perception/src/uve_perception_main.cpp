#include <rclcpp/rclcpp.hpp>
#include "uve_perception.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UvePerception>());
    rclcpp::shutdown();
    return 0;
}