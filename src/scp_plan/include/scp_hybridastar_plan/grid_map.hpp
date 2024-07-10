#ifndef GRID_MAP_HPP
#define GRID_MAP_HPP

#include <rclcpp/rclcpp.hpp>
#include <map>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <opencv4/opencv2/opencv.hpp>

#include "type.hpp"
#include "element.hpp"

using geometry_msgs::msg::Pose2D;
using std::multimap;

namespace scp
{

class GridNode
{
public:
    
public:
    GridNodeIndex index;
    Pose2D pose;
    GridNodeSearchInfo search_info;
public:
    GridNode();
    GridNode(GridNodeIndex index);
    GridNode(GridNodeIndex index, Pose2D pose);
    GridNode(const GridNode& node);
    GridNode& operator=(const GridNode& node);
};

class GridMap
{
public:
public:
    GridNode*** gridMap = nullptr;
    bool** gridMapOccupied = nullptr;
    // double** gridMapDistance = nullptr;
    std::set<int>** gridMapCollision = nullptr;

    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    double positionResolution = 0;
    double yawResolution = 0;
    int yawStep = 0;
    std::vector<double> yawList;
    int width = 0, height = 0, depth = 0;
    int oriX = 0, oriY = 0;

    std::map<int, std::vector<std::pair<int,int>>> elementOccupied;
    std::map<int, std::vector<std::pair<int,int>>> elementCollision;
public:
    GridMap();
    GridMap(double minX, double minY, double maxX, double maxY, double positionResolution, int yaeStep);
    GridMap(const GridMap& map);
    ~GridMap();

    void putElement(Element& element);
    void removeElement(Element& element);

    __attribute__((unused)) void setOccupied(int x, int y, bool occupied);
    __attribute__((unused)) void setDistance(int x, int y, double distance);

    GridMap& operator=(const GridMap& map);
    GridState operator()(Pose2D &pose);
    GridState operator()(GridNodeIndex &index);
    GridState operator()(int x, int y, int z);

    void toMsg(nav_msgs::msg::OccupancyGrid& msg);
};

} // namespace scp

#endif // GRID_MAP_HPP