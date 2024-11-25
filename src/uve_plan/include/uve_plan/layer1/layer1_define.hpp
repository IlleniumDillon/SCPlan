#ifndef LAYER1_DEFINE_HPP
#define LAYER1_DEFINE_HPP

#include "uvs_message/srv/uv_query_world.hpp"
#include "uve_message/msg/uve_dynamic_status_list.hpp"

#include <string>
#include <map>
#include <vector>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Eigen>


namespace layer1
{

using WorldDscp = uvs_message::srv::UvQueryWorld::Response;

enum Layer1GraphNodeFlag
{
    NOT_VISITED,
    IN_OPENSET,
    IN_CLOSESET
};

class Layer1GraphNodeCost
{
public:
    double distance = 0;
    double rotation = 0;
public:
    Layer1GraphNodeCost(){};
    Layer1GraphNodeCost(double distance, double rotation) : distance(distance), rotation(rotation){};
    ~Layer1GraphNodeCost() = default;
    Layer1GraphNodeCost(Layer1GraphNodeCost& other) = default;
    Layer1GraphNodeCost(const Layer1GraphNodeCost& other) = default;
    Layer1GraphNodeCost& operator=(const Layer1GraphNodeCost& cost)
    {
        distance = cost.distance;
        rotation = cost.rotation;
        return *this;
    }

    bool operator==(const Layer1GraphNodeCost& cost) const
    {
        return distance == cost.distance && rotation == cost.rotation;
    }
    bool operator!=(const Layer1GraphNodeCost& cost) const
    {
        return distance != cost.distance || rotation != cost.rotation;
    }
    bool operator<(const Layer1GraphNodeCost& cost) const
    {
        return distance < cost.distance || (distance == cost.distance && rotation < cost.rotation);
    }
    bool operator>(const Layer1GraphNodeCost& cost) const
    {
        return distance > cost.distance || (distance == cost.distance && rotation > cost.rotation);
    }
    Layer1GraphNodeCost operator+(const Layer1GraphNodeCost& cost) const
    {
        return Layer1GraphNodeCost(distance + cost.distance, rotation + cost.rotation);
    }
    Layer1GraphNodeCost operator-(const Layer1GraphNodeCost& cost) const
    {
        return Layer1GraphNodeCost(distance - cost.distance, rotation - cost.rotation);
    }
};

class Layer1GraphNode
{
public:
    Layer1GraphNode()
    {
        state = cv::Point3d(0, 0, 0);
        index = cv::Point3i(0, 0, 0);
        g = Layer1GraphNodeCost();
        h = Layer1GraphNodeCost();
        f = Layer1GraphNodeCost();
        parent = nullptr;
        flag = NOT_VISITED;
        collision_static = false;
        collision_dynamic = false;
        vw = cv::Point2d(0, 0);
    };
    ~Layer1GraphNode() = default;
    Layer1GraphNode(Layer1GraphNode& other) = default;
    Layer1GraphNode(const Layer1GraphNode& other) = default;
    Layer1GraphNode& operator=(const Layer1GraphNode& node)
    {
        state = node.state;
        index = node.index;
        g = node.g;
        h = node.h;
        f = node.f;
        parent = node.parent;
        flag = node.flag;
        collision_static = node.collision_static;
        collision_dynamic = node.collision_dynamic;
        vw = node.vw;
        return *this;
    }
    bool operator==(const Layer1GraphNode& node) const
    {
        return index == node.index;
    }
public:
    cv::Point3d state;
    cv::Point3i index;
    Layer1GraphNodeCost g, h, f;
    Layer1GraphNode* parent;
    std::multimap<Layer1GraphNodeCost, Layer1GraphNode*>::iterator it;
    Layer1GraphNodeFlag flag;
    bool collision_static;
    bool collision_dynamic;
    cv::Point2d vw;
};

class Layer1GridGraph
{
public:
    Layer1GridGraph(){}
    Layer1GridGraph(WorldDscp& world, cv::Point3d res);
    Layer1GridGraph(std::string path);

    ~Layer1GridGraph();
    Layer1GridGraph(Layer1GridGraph& other) = delete;
    Layer1GridGraph(const Layer1GridGraph& other) = delete;
    Layer1GridGraph& operator=(const Layer1GridGraph& other) = delete;


    Layer1GraphNode* operator()(int x, int y, int z);
    Layer1GraphNode* operator()(double x, double y, double z);

    bool setStaticCollision(double x, double y, double theta);

    void updateDynamic(uve_message::msg::UveDynamicStatusList& nstate);
    void copyFrom(Layer1GridGraph& other);

    void save(std::string path);

    void ignoreDynamicCollision(std::string name);
    // void setGround
public:
    cv::Point3d origin;
    cv::Point3d resolution;
    cv::Point3i size;
    cv::Point3d state_min;
    cv::Point3d state_max;

    std::vector<Layer1GraphNode*> nodes;

    std::vector<cv::Point2f> ground;
    std::vector<std::vector<cv::Point2f>> static_obstacles;
    std::vector<std::vector<cv::Point2f>> dynamic_shapes;
    std::vector<std::vector<cv::Point3f>> dynamic_collisions_zero;
    std::vector<std::vector<cv::Point3f>> dynamic_collisions_last;
    std::map<std::string, int> dynamic_map;
    std::vector<cv::Point3d> dynamic_pose;
    std::vector<cv::Point2f> agent_shape;

    double XY;
};

}   // namespace layer1

#endif // LAYER1_DEFINE_HPP