#include <rclcpp/rclcpp.hpp>
#include "uve_mapclient.hpp"

int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UveMapClient>());
    rclcpp::shutdown();
    return 0;
}