#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

int main(int argc, char * argv[])
{
    try {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("scp_simulate");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Package share directory: %s", package_share_directory.c_str());
        // 使用package_share_directory
    } catch (const ament_index_cpp::PackageNotFoundError& e) {
        std::cerr << "Package not found: " << e.what() << std::endl;
        // 处理包未找到的情况
    }
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rclcpp::Node>("scp_plan_justmove"));
    rclcpp::shutdown();
    return 0;
}