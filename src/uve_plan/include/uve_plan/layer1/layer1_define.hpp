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
        collision_static = true;
        collision_dynamic = true;
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
    Layer1GridGraph(WorldDscp& world, cv::Point3d res)
    {
        origin = cv::Point3d(   world.ground.origin.x, 
                                world.ground.origin.y, 
                                world.ground.origin.z);
        resolution = res;

        for (auto& point : world.ground.shape.points)
        {
            ground.push_back(cv::Point2f(point.x, point.y));
        }
        float max_agent_edge = 0;
        for (auto& point : world.agents[0].shape.points)
        {
            agent_shape.push_back(cv::Point2f(point.x, point.y));
            auto temp = agent_shape[0] - agent_shape.back();
            max_agent_edge = std::max(max_agent_edge, sqrt(temp.x * temp.x + temp.y * temp.y));
        }
        for (auto& obstacle : world.obstacles)
        {
            std::vector<cv::Point2f> points;
            for (auto& point : obstacle.shape.points)
            {
                points.push_back(cv::Point2f(point.x, point.y));
            }
            static_obstacles.push_back(points);
        }
        for (auto& dynamic : world.cargos)
        {
            std::vector<cv::Point2f> points;
            float max_cargo_edge = 0;
            for (auto& point : dynamic.shape.points)
            {
                points.push_back(cv::Point2f(point.x, point.y));
                auto temp = points[0] - points.back();
                max_cargo_edge = std::max(max_cargo_edge, sqrt(temp.x * temp.x + temp.y * temp.y));
            }
            dynamic_shapes.push_back(points);
            dynamic_map[dynamic.name] = dynamic_shapes.size() - 1;

            float half_edge = (max_cargo_edge + max_agent_edge) / 2;
            std::vector<cv::Point3f> zero;
            for (float _x = -half_edge; _x <= half_edge; _x += resolution.x)
            {
                for (float _y = -half_edge; _y <= half_edge; _y += resolution.y)
                {
                    for (float _z = -M_PI; _z <= M_PI; _z += resolution.z)
                    {
                        std::vector<cv::Point2f> shape = agent_shape;
                        for (auto & p : shape)
                        {
                            p = cv::Point2f(p.x * cos(_z) - p.y * sin(_z) + _x,
                                            p.x * sin(_z) + p.y * cos(_z) + _y);
                        }
                        std::vector<cv::Point2f> temp;
                        if (cv::intersectConvexConvex(shape, dynamic_shapes.back(), temp))
                        {
                            zero.push_back(cv::Point3f(_x, _y, _z));
                        }
                    }
                }
            }
            dynamic_collisions_zero.push_back(zero);
        }
        dynamic_collisions_last = dynamic_collisions_zero;


        float min_x = 1e9, min_y = 1e9, min_z = -M_PI;
        float max_x = -1e9, max_y = -1e9, max_z = M_PI;
        for (auto& point : world.ground.shape.points)
        {
            min_x = std::min(min_x, point.x + world.ground.origin.x);
            min_y = std::min(min_y, point.y + world.ground.origin.y);
            max_x = std::max(max_x, point.x + world.ground.origin.x);
            max_y = std::max(max_y, point.y + world.ground.origin.y);
        }
        min_x -= 5 * world.ground.resolution_x;
        min_y -= 5 * world.ground.resolution_y;
        max_x += 5 * world.ground.resolution_x;
        max_y += 5 * world.ground.resolution_y;

        state_min = cv::Point3d(min_x, min_y, min_z);
        state_max = cv::Point3d(max_x, max_y, max_z);

        size = cv::Point3i((state_max.x - state_min.x) / resolution.x,
                           (state_max.y - state_min.y) / resolution.y,
                           (state_max.z - state_min.z) / resolution.z);
        
        XY = size.x * size.y;

        nodes.resize(size.x * size.y * size.z);
        for (int z = 0; z < size.z; z++)
        {
            for (int y = 0; y < size.y; y++)
            {
                for (int x = 0; x < size.x; x++)
                {
                    nodes[x + y * size.x + z * XY] = new Layer1GraphNode();
                    nodes[x + y * size.x + z * XY]->state = cv::Point3d(state_min.x + x * resolution.x,
                                                                        state_min.y + y * resolution.y,
                                                                        state_min.z + z * resolution.z);
                    nodes[x + y * size.x + z * XY]->index = cv::Point3i(x, y, z);
                    nodes[x + y * size.x + z * XY]->collision_static = setStaticCollision(nodes[x + y * size.x + z * XY]->state.x,
                                                                                        nodes[x + y * size.x + z * XY]->state.y,
                                                                                        nodes[x + y * size.x + z * XY]->state.z);
                }
            }
        }

    }

    ~Layer1GridGraph()
    {
        for (auto node : nodes)
        {
            delete node;
        }
    }
    Layer1GridGraph(Layer1GridGraph& other) = delete;
    Layer1GridGraph(const Layer1GridGraph& other) = delete;
    Layer1GridGraph& operator=(const Layer1GridGraph& other) = delete;


    Layer1GraphNode* operator()(int x, int y, int z)
    {
        if (x < 0 || x >= size.x || y < 0 || y >= size.y || z < 0 || z >= size.z)
        {
            return nullptr;
        }
        return nodes[x + y * size.x + z * XY];
    }
    Layer1GraphNode* operator()(double x, double y, double z)
    {
        int ix = (x - state_min.x) / resolution.x;
        int iy = (y - state_min.y) / resolution.y;
        int iz = (z - state_min.z) / resolution.z;
        iz %= size.z;
        if (iz < 0)
        {
            iz += size.z;
        }
        return (*this)(ix, iy, iz);
    }

    bool setStaticCollision(double x, double y, double theta)
    {
        std::vector<cv::Point2f> shape = agent_shape;
        for (auto & p : shape)
        {
            p = cv::Point2f(p.x * cos(theta) - p.y * sin(theta) + x,
                            p.x * sin(theta) + p.y * cos(theta) + y);

            if (cv::pointPolygonTest(ground, p, false) <= 0)
            {
                return true;
            }
        }

        std::vector<cv::Point2f> temp;
        for (auto & obstacle : static_obstacles)
        {
            if (cv::intersectConvexConvex(shape, obstacle, temp))
            {
                return true;
            }
        }

        return false;
    }

    void updateDynamic(uve_message::msg::UveDynamicStatusList& nstate)
    {
        for (auto& last : dynamic_collisions_last)
        {
            for (auto& point : last)
            {
                auto node = (*this)(point.x, point.y, point.z);
                if (node)
                {
                    node->collision_dynamic = false;
                }
            }
        }
        for (auto& state : nstate.list)
        {
            auto it = dynamic_map.find(state.name);
            if (it == dynamic_map.end())
            {
                continue;
            }
            auto& last = dynamic_collisions_last[it->second];
            auto& zero = dynamic_collisions_zero[it->second];
            for (int i = 0; i < last.size(); i++)
            {
                last[i] = cv::Point3f(
                    state.pose.x + std::cos(state.pose.theta) * zero[i].x - std::sin(state.pose.theta) * zero[i].y,
                    state.pose.y + std::sin(state.pose.theta) * zero[i].x + std::cos(state.pose.theta) * zero[i].y,
                    state.pose.theta + zero[i].z
                );
                auto node = (*this)(last[i].x, last[i].y, last[i].z);
                if (node)
                {
                    node->collision_dynamic = true;
                }
            }
        }
    }

    void copyFrom(Layer1GridGraph& other)
    {
        origin = other.origin;
        resolution = other.resolution;
        size = other.size;
        state_min = other.state_min;
        state_max = other.state_max;
        nodes.resize(size.x * size.y * size.z);
        for (int z = 0; z < size.z; z++)
        {
            for (int y = 0; y < size.y; y++)
            {
                for (int x = 0; x < size.x; x++)
                {
                    nodes[x + y * size.x + z * XY] = new Layer1GraphNode();
                    nodes[x + y * size.x + z * XY] = other.nodes[x + y * size.x + z * XY];
                }
            }
        }
        ground = other.ground;
        static_obstacles = other.static_obstacles;
        dynamic_shapes = other.dynamic_shapes;
        dynamic_collisions_zero = other.dynamic_collisions_zero;
        dynamic_collisions_last = other.dynamic_collisions_last;
        dynamic_map = other.dynamic_map;
        agent_shape = other.agent_shape;
        XY = other.XY;
    }
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
    std::vector<cv::Point2f> agent_shape;

    double XY;
};

}   // namespace layer1

#endif // LAYER1_DEFINE_HPP