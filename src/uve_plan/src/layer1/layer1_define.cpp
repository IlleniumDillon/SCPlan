#include "layer1_define.hpp"

#include "json/json.h"

using namespace layer1;

Layer1GridGraph::Layer1GridGraph(WorldDscp& world, cv::Point3d res)
{
    origin = cv::Point3d(   world.ground.origin.x, 
                            world.ground.origin.y, 
                            world.ground.origin.z);
    resolution = res;

    float min_x = 1e9, min_y = 1e9, min_z = -M_PI;
    float max_x = -1e9, max_y = -1e9, max_z = M_PI;
    for (auto& point : world.ground.shape.points)
    {
        ground.emplace_back(cv::Point2f(point.x, point.y));
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

    float max_agent_edge = 0;
    for (auto& point : world.agents[0].shape.points)
    {
        agent_shape.emplace_back(cv::Point2f(point.x, point.y));
        auto temp = agent_shape[0] - agent_shape.back();
        max_agent_edge = std::max(max_agent_edge, sqrt(temp.x * temp.x + temp.y * temp.y));
    }
    for (auto& obstacle : world.obstacles)
    {
        std::vector<cv::Point2f> points;
        for (auto& point : obstacle.shape.points)
        {
            points.emplace_back(cv::Point2f(point.x * cos(obstacle.pose.theta) - point.y * sin(obstacle.pose.theta) + obstacle.pose.x,
                                        point.x * sin(obstacle.pose.theta) + point.y * cos(obstacle.pose.theta) + obstacle.pose.y));
        }
        static_obstacles.emplace_back(points);
    }
    for (auto& dynamic : world.cargos)
    {
        std::vector<cv::Point2f> points;
        float max_cargo_edge = 0;
        for (auto& point : dynamic.shape.points)
        {
            points.emplace_back(cv::Point2f(point.x, point.y));
            auto temp = points[0] - points.back();
            max_cargo_edge = std::max(max_cargo_edge, sqrt(temp.x * temp.x + temp.y * temp.y));
        }
        dynamic_shapes.emplace_back(points);
        dynamic_map[dynamic.name] = dynamic_shapes.size() - 1;

        float half_edge = (max_cargo_edge + max_agent_edge) / 2;
        std::vector<cv::Point3f> zero;
        for (float _x = -half_edge; _x <= half_edge; _x += resolution.x)
        {
            for (float _y = -half_edge; _y <= half_edge; _y += resolution.y)
            {
                for (int z_ = 0; z_ < size.z * 2; z_++)
                {
                    float _z = state_min.z + z_ * resolution.z / 2;
                    std::vector<cv::Point2f> shape = agent_shape;
                    for (auto & p : shape)
                    {
                        p = cv::Point2f(p.x * cos(_z) - p.y * sin(_z) + _x,
                                        p.x * sin(_z) + p.y * cos(_z) + _y);
                    }
                    std::vector<cv::Point2f> temp;
                    if (cv::intersectConvexConvex(shape, dynamic_shapes.back(), temp))
                    {
                        zero.emplace_back(cv::Point3f(_x, _y, _z));
                    }
                }
            }
        }
        dynamic_collisions_zero.emplace_back(zero);
    }
    dynamic_collisions_last = dynamic_collisions_zero;

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

Layer1GridGraph::Layer1GridGraph(std::string path)
{
    std::ifstream ifs(path);
    if (!ifs.is_open())
    {
        throw std::runtime_error("file not found");
    }
    Json::Reader reader;
    Json::Value root;
    if (!reader.parse(ifs, root, false))
    {
        throw std::runtime_error("json parse error");
    }
    ifs.close();
    // origin
    Json::Value origin = root["origin"];
    this->origin = cv::Point3d(origin[0].asDouble(), origin[1].asDouble(), origin[2].asDouble());
    // resolution
    Json::Value resolution = root["resolution"];
    this->resolution = cv::Point3d(resolution[0].asDouble(), resolution[1].asDouble(), resolution[2].asDouble());
    // size
    Json::Value size = root["size"];
    this->size = cv::Point3i(size[0].asInt(), size[1].asInt(), size[2].asInt());
    this->XY = this->size.x * this->size.y;
    // state_min
    Json::Value state_min = root["state_min"];
    this->state_min = cv::Point3d(state_min[0].asDouble(), state_min[1].asDouble(), state_min[2].asDouble());
    // state_max
    Json::Value state_max = root["state_max"];
    this->state_max = cv::Point3d(state_max[0].asDouble(), state_max[1].asDouble(), state_max[2].asDouble());

    nodes.resize(this->size.x * this->size.y * this->size.z);
    for (int z = 0; z < this->size.z; z++)
    {
        for (int y = 0; y < this->size.y; y++)
        {
            for (int x = 0; x < this->size.x; x++)
            {
                nodes[x + y * this->size.x + z * this->XY] = new Layer1GraphNode();
                nodes[x + y * this->size.x + z * this->XY]->state = cv::Point3d(this->state_min.x + x * this->resolution.x,
                                                                    this->state_min.y + y * this->resolution.y,
                                                                    this->state_min.z + z * this->resolution.z);
                nodes[x + y * this->size.x + z * this->XY]->index = cv::Point3i(x, y, z);
            }
        }
    }

    // static_collision
    Json::Value static_collision = root["static_collision"];
    for (auto & v : static_collision)
    {
        auto node = (*this)(v[0].asInt(), v[1].asInt(), v[2].asInt());
        if (node)
        {
            node->collision_static = true;
        }
    }
    // dynamic_map
    Json::Value dynamic_map = root["dynamic_map"];
    for (auto & v : dynamic_map)
    {
        this->dynamic_map[v[0].asString()] = v[1].asInt();
    }
    // dynamic_collision_zero
    Json::Value dynamic_collision_zero = root["dynamic_collision_zero"];
    std::vector<cv::Point3f> zero;
    for (auto & v : dynamic_collision_zero)
    {
        zero.emplace_back(cv::Point3f(v[0].asDouble(), v[1].asDouble(), v[2].asDouble()));
    }
    for (int i = 0; i < dynamic_map.size(); i++)
    {
        this->dynamic_collisions_zero.emplace_back(zero);
    }
    this->dynamic_collisions_last = this->dynamic_collisions_zero;
}

Layer1GridGraph::~Layer1GridGraph()
{
    for (auto node : nodes)
    {
        delete node;
    }
}

Layer1GraphNode *Layer1GridGraph::operator()(int x, int y, int z)
{
    if (x < 0 || x >= size.x || y < 0 || y >= size.y || z < 0 || z >= size.z)
    {
        return nullptr;
    }
    return nodes[x + y * size.x + z * XY];
}

Layer1GraphNode *Layer1GridGraph::operator()(double x, double y, double z)
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

bool Layer1GridGraph::setStaticCollision(double x, double y, double theta)
{
    std::vector<cv::Point2f> shape = agent_shape;
    for (auto & p : shape)
    {
        p = cv::Point2f(p.x * cos(theta) - p.y * sin(theta) + x,
                        p.x * sin(theta) + p.y * cos(theta) + y);

        if (cv::pointPolygonTest(ground, p, false) <= 0)
        {
            // std::cout << "g";
            return true;
        }
    }

    std::vector<cv::Point2f> temp;
    for (auto & obstacle : static_obstacles)
    {
        if (cv::intersectConvexConvex(shape, obstacle, temp))
        {
            // std::cout << "o";
            return true;
        }
    }
    // std::cout << "f";
    return false;
}

void Layer1GridGraph::updateDynamic(uve_message::msg::UveDynamicStatusList &nstate)
{
    if (dynamic_pose.size() < nstate.list.size())
    {
        dynamic_pose.resize(nstate.list.size());
    }
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
        dynamic_pose[it->second] = cv::Point3d(state.pose.x, state.pose.y, state.pose.theta);
        auto& last = dynamic_collisions_last[it->second];
        auto& zero = dynamic_collisions_zero[it->second];
        // int cont = 0;
        for (int i = 0; i < last.size(); i++)
        {
            last[i] = cv::Point3f(
                state.pose.x + std::cos(state.pose.theta) * zero[i].x - std::sin(state.pose.theta) * zero[i].y,
                state.pose.y + std::sin(state.pose.theta) * zero[i].x + std::cos(state.pose.theta) * zero[i].y,
                state.pose.theta + zero[i].z
            );

            if (last[i].z > M_PI)
            {
                last[i].z -= 2 * M_PI;
            }
            else if (last[i].z <= -M_PI)
            {
                last[i].z += 2 * M_PI;
            }
            
            auto node = (*this)(last[i].x, last[i].y, last[i].z);
            if (node)
            {
                // cont++;
                node->collision_dynamic = true;
            }
            
        }
        // std::cout << cont <<  " " << last.size() << std::endl;
    }
}

void Layer1GridGraph::copyFrom(Layer1GridGraph &other)
{
    origin = other.origin;
    resolution = other.resolution;
    size = other.size;
    XY = other.XY;
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
    dynamic_pose = other.dynamic_pose;
    agent_shape = other.agent_shape;
}

void Layer1GridGraph::save(std::string path)
{
    Json::Value root;
    // origin
    Json::Value origin;
    origin.append(this->origin.x);
    origin.append(this->origin.y);
    origin.append(this->origin.z);
    root["origin"] = origin;
    // resolution
    Json::Value resolution;
    resolution.append(this->resolution.x);
    resolution.append(this->resolution.y);
    resolution.append(this->resolution.z);
    root["resolution"] = resolution;
    // size
    Json::Value size;
    size.append(this->size.x);
    size.append(this->size.y);
    size.append(this->size.z);
    root["size"] = size;
    // state_min
    Json::Value state_min;
    state_min.append(this->state_min.x);
    state_min.append(this->state_min.y);
    state_min.append(this->state_min.z);
    root["state_min"] = state_min;
    // state_max
    Json::Value state_max;
    state_max.append(this->state_max.x);
    state_max.append(this->state_max.y);
    state_max.append(this->state_max.z);
    root["state_max"] = state_max;
    // static_collision
    Json::Value static_collision;
    for (int z = 0; z < this->size.z; z++)
    {
        for (int y = 0; y < this->size.y; y++)
        {
            for (int x = 0; x < this->size.x; x++)
            {
                auto node = (*this)(x, y, z);
                if (node->collision_static)
                {
                    Json::Value v;
                    v.append(x);
                    v.append(y);
                    v.append(z);
                    static_collision.append(v);
                }
            }
        }
    }
    root["static_collision"] = static_collision;
    // dynamic_map
    Json::Value dynamic_map;
    for (auto & v : this->dynamic_map)
    {
        Json::Value temp;
        temp.append(v.first);
        temp.append(v.second);
        dynamic_map.append(temp);
    }
    root["dynamic_map"] = dynamic_map;
    // dynamic_collision_zero
    Json::Value dynamic_collision_zero;
    for (auto & v : this->dynamic_collisions_zero[0])
    {
        Json::Value temp;
        temp.append(v.x);
        temp.append(v.y);
        temp.append(v.z);
        dynamic_collision_zero.append(temp);
    }
    root["dynamic_collision_zero"] = dynamic_collision_zero;

    std::ofstream ofs(path);
    // if (!ofs.is_open())
    // {
    //     throw std::runtime_error("file not found");
    // }
    ofs << root;
    ofs.close();
}

void Layer1GridGraph::ignoreDynamicCollision(std::string name)
{
    auto it = dynamic_map.find(name);
    if (it == dynamic_map.end())
    {
        return;
    }
    for (auto& point : dynamic_collisions_last[it->second])
    {
        auto node = (*this)(point.x, point.y, point.z);
        if (node)
        {
            node->collision_dynamic = false;
        }
    }
}
