#include "connect_graph.hpp"

using namespace scp;

ConnectGraph::ConnectGraph()
{
}

void ConnectGraph::config(double radius)
{
    this->radius = radius;
}

void ConnectGraph::convertGridMap(GridMap &grid_map)
{
    //RCLCPP_INFO(rclcpp::get_logger("scp_plan"), "Convert grid map to pix map.");
    pix_map = PixMap(grid_map);
    //pix_map.showPixMap();
    //RCLCPP_INFO(rclcpp::get_logger("scp_plan"), "Closing pix map: %d.", (int)(radius / pix_map.resolution));
    pix_map.closing((int)(radius / pix_map.resolution));
    //pix_map.showPixMap();
    //RCLCPP_INFO(rclcpp::get_logger("scp_plan"), "Label connect domain.");
    pix_map.labelConnectDomain();
    //pix_map.showPixMap();
    //RCLCPP_INFO(rclcpp::get_logger("scp_plan"), "Convert grid map to pix map done.");
}
