#ifndef CONNECT_GRAPH_HPP
#define CONNECT_GRAPH_HPP

#include <rclcpp/rclcpp.hpp>

#include "graph_typedef.hpp"
#include "grid_map.hpp"
#include "pix_map.hpp"

#include <opencv2/opencv.hpp>

namespace scp
{
class ConnectGraph
{
public:
    PixMap pix_map;
    double radius = 0.5;
public:
    ConnectGraph();
public:
    void config(double radius);
    void convertGridMap(GridMap& grid_map);
};
} // namespace scp

#endif // CONNECT_GRAPH_HPP