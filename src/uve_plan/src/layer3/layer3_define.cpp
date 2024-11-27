#include "layer3_define.hpp"
#include <string>

using namespace layer3;
using namespace layer2;
using namespace layer1;

void Layer3PixMap::fromGridGraph(layer1::Layer1GridGraph &graph, 
        uve_message::msg::UveDynamicStatusList &dynamic_state,
        uvs_message::srv::UvQueryWorld::Response& world)
{
    // copy the basic information
    origin = cv::Point2d(graph.origin.x, graph.origin.y);
    resolution = cv::Point2d(graph.resolution.x, graph.resolution.y);
    size = cv::Point2d(graph.size.x, graph.size.y);
    state_min = cv::Point2d(graph.state_min.x, graph.state_min.y);
    state_max = cv::Point2d(graph.state_max.x, graph.state_max.y);

    // initialize the domain map
    int pix_left = size.x * size.y;
    domainMap.clear();
    domainMap.resize(pix_left);
    for (int i = 0; i < size.x; i++)
    {
        for (int j = 0; j < size.y; j++)
        {
            int count = 0;
            for (int k = 0; k < graph.size.z; k++)
            {
                auto node = graph(i, j, k);
                if (node->collision_static || node->collision_dynamic)
                {
                    count++;
                }
            }
            if (count == graph.size.z)
            {
                *(*this)(i, j) = -2;
                pix_left--;
            }
            else
            {
                *(*this)(i, j) = -1;
            }
        }
    }

    // set the domain map
    int cur_domain = 0;
    while (pix_left > 0)
    {
        // find the first unknown pixel
        int x = -1, y = -1;
        for (int i = 0; i < size.x; i++)
        {
            for (int j = 0; j < size.y; j++)
            {
                if (*(*this)(i, j) == -1)
                {
                    x = i;
                    y = j;
                    break;
                }
            }
        }
        if (x == -1)
        {
            break;
        }

        // BFS
        std::queue<std::pair<int, int>> q;
        *(*this)(x, y) = cur_domain;
        pix_left--;
        q.push(std::make_pair(x, y));
        while (!q.empty())
        {
            auto p = q.front();
            q.pop();

            for (int i = -1; i <= 1; i++)
            {
                for (int j = -1; j <= 1; j++)
                {
                    if (i == 0 && j == 0)
                    {
                        continue;
                    }
                    if (p.first + i < 0 || p.first + i >= size.x || p.second + j < 0 || p.second + j >= size.y)
                    {
                        continue;
                    }
                    if (*(*this)(p.first + i, p.second + j) == -1)
                    {
                        *(*this)(p.first + i, p.second + j) = cur_domain;
                        pix_left--;
                        q.push(std::make_pair(p.first + i, p.second + j));
                    }
                }
            }
        }
        cur_domain++;
    }
    numOfDomain = cur_domain;

    // generate the edges
    edges.clear();
    for (int i = 0; i < dynamic_state.list.size(); i++)
    {
        std::string Cname = dynamic_state.list[i].name;
        auto targetCargoIt = graph.dynamic_map.find(Cname);
        if (targetCargoIt == graph.dynamic_map.end())
        {
            continue;
        }
        cv::Point3d Cpos = cv::Point3d(dynamic_state.list[i].pose.x, dynamic_state.list[i].pose.y, dynamic_state.list[i].pose.theta);
        auto& targetCargo = world.cargos[targetCargoIt->second];
        std::vector<double> cargo_anchors_theta;
        for (auto& p : targetCargo.anchors)
        {
            cargo_anchors_theta.push_back(
                std::atan2(p.y, p.x)
            );
        }
        double agent_anchor_center = std::sqrt(std::pow(world.agents[0].anchors[0].x, 2) + std::pow(world.agents[0].anchors[0].y, 2));
        std::vector<cv::Point2d> CAstart_list;
        for (int j = 0; j < targetCargo.anchors.size(); j++)
        {
            double theta = Cpos.z + cargo_anchors_theta[j];
            double x = Cpos.x + targetCargo.anchors[j].x * std::cos(Cpos.z) - targetCargo.anchors[j].y * std::sin(Cpos.z)
                            + agent_anchor_center * std::cos(theta);
            double y = Cpos.y + targetCargo.anchors[j].x * std::sin(Cpos.z) + targetCargo.anchors[j].y * std::cos(Cpos.z)
                            + agent_anchor_center * std::sin(theta);
            auto p = (*this)(x, y);
            if (p != nullptr)
            {
                if (*p >= 0)
                {
                    CAstart_list.push_back(cv::Point2d(x, y));
                }
            }
        }
        if (CAstart_list.size() == 2)
        {
            if (*(*this)(CAstart_list[0].x, CAstart_list[0].y) != *(*this)(CAstart_list[1].x, CAstart_list[1].y))
            {
                edges.push_back(
                    Layer3ConnectEdge{
                        edges.size(),
                        *(*this)(CAstart_list[0].x, CAstart_list[0].y),
                        *(*this)(CAstart_list[1].x, CAstart_list[1].y),
                        Cname,
                        std::atan2(CAstart_list[1].y - CAstart_list[0].y, CAstart_list[1].x - CAstart_list[0].x)
                    }
                );
                domain_edges[edges.back().domain1].insert(edges.size() - 1);
                domain_edges[edges.back().domain2].insert(edges.size() - 1);
            }
        }
    }
}

int* Layer3PixMap::operator()(int x, int y)
{
    if (x < 0 || x >= size.x || y < 0 || y >= size.y)
    {
        return nullptr;
    }
    return &domainMap[y * size.x + x];
}

int* Layer3PixMap::operator()(double x, double y)
{
    int i = (x - state_min.x) / resolution.x;
    int j = (y - state_min.y) / resolution.y;
    return (*this)(i, j);
}

void Layer3SearchGraph::fromPixMap(Layer3PixMap &map, std::vector<cv::Point3d> &checkPoints_ref)
{
    // set the basic information
    edge_num = map.edges.size();
    checkPoint_num = checkPoints_ref.size();
    checkPoints = checkPoints_ref;

    // initialize the graph
    nodes.clear();
    nodes.resize(edge_num * checkPoint_num);
    for (int i = 0; i < edge_num; i++)
    {
        for (int j = 0; j < checkPoint_num; j++)
        {
            nodes[i * checkPoint_num + j].index = std::make_pair(i, j);
        }
    }
}

Layer3SearchNode *Layer3SearchGraph::operator()(int edge_id, int checkPoint_id)
{
    if (edge_id < 0 || edge_id >= edge_num || checkPoint_id < 0 || checkPoint_id >= checkPoint_num)
    {
        return nullptr;
    }
    return &nodes[edge_id * checkPoint_num + checkPoint_id];
}

Layer3SearchNode *Layer3SearchGraph::operator()(std::pair<int, int> index)
{
    return (*this)(index.first, index.second);
}
