#ifndef TYPE_HPP
#define TYPE_HPP

#include <rclcpp/rclcpp.hpp>
#include <map>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <opencv4/opencv2/opencv.hpp>

#include "element.hpp"

using geometry_msgs::msg::Pose2D;
using std::multimap;

namespace scp
{

enum GridNodeState
{
    NOT_VISITED,
    IN_OPENSET,
    IN_CLOSESET
};

struct GridNodeIndex
{
    int x = -1;
    int y = -1;
    int z = -1;

    bool operator==(const GridNodeIndex& index) const
    {
        return x == index.x && y == index.y && z == index.z;
    }
};

class GridCost
{
public:
    double distance = 0;
    double rotation = 0;
public:
    GridCost(){};
    GridCost(double distance, double rotation) : distance(distance), rotation(rotation){};
    bool operator==(const GridCost& cost) const
    {
        return distance == cost.distance && rotation == cost.rotation;
    }
    bool operator!=(const GridCost& cost) const
    {
        return distance != cost.distance || rotation != cost.rotation;
    }
    bool operator<(const GridCost& cost) const
    {
        return distance < cost.distance || (distance == cost.distance && rotation < cost.rotation);
    }
    bool operator>(const GridCost& cost) const
    {
        return distance > cost.distance || (distance == cost.distance && rotation > cost.rotation);
    }
    GridCost operator+(const GridCost& cost) const
    {
        return GridCost(distance + cost.distance, rotation + cost.rotation);
    }
    GridCost operator-(const GridCost& cost) const
    {
        return GridCost(distance - cost.distance, rotation - cost.rotation);
    }
};

class GridNode;

struct GridNodeSearchInfo
{
    GridCost g;
    GridCost h;
    GridCost f;
    GridNode* parent = nullptr;
    multimap<GridCost, GridNode*>::iterator it;
    GridNodeState state = NOT_VISITED;
};

struct GridState
{
    GridNode* node = nullptr;
    bool occupied = false;
    double distance = 0;
};

struct PlanResult
{
    bool success = false;
    int iterations = 0;
    double planTime = 0;
    double cost = 0;
    std::vector<Pose2D> path;
};

} // namespace scp

#endif // TYPE_HPP