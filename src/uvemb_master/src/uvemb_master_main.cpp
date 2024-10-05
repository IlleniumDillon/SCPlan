#include <rclcpp/rclcpp.hpp>
#include <sys/unistd.h>
#include "uvemb_master.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UvEmbMaster>());    
    rclcpp::shutdown();
    return 0;
}