#ifndef LAYER3_DEFINE_HPP
#define LAYER3_DEFINE_HPP

#include <string>
#include <map>
#include <vector>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Eigen>

#include "layer1/layer1_plan.hpp"
#include "layer2/layer2_plan.hpp"

namespace layer3
{

using Layer3GraphNodeFlag = layer1::Layer1GraphNodeFlag;

class Layer3ConnectEdge
{
public:
    Layer3ConnectEdge(){}
    Layer3ConnectEdge(int id, int domain1, int domain2, std::string name, double direction)
        : edge_id(id), domain1(domain1), domain2(domain2), name(name), direction(direction){}
    ~Layer3ConnectEdge() = default;

    int otherDomain(int domain)
    {
        if (domain == domain1)
        {
            return domain2;
        }
        if (domain == domain2)
        {
            return domain1;
        }
        return -1;
    }
public:
    int edge_id;
    int domain1;
    int domain2;
    std::string name;
    double direction;
};

class Layer3PixMap
{
public:
    Layer3PixMap(){}
    ~Layer3PixMap() = default;

    Layer3PixMap(Layer3PixMap& other) = delete;
    Layer3PixMap(const Layer3PixMap& other) = delete;
    Layer3PixMap& operator=(const Layer3PixMap& other) = delete;

    void fromGridGraph(layer1::Layer1GridGraph& graph, 
        uve_message::msg::UveDynamicStatusList& dynamic_state,
        uvs_message::srv::UvQueryWorld::Response& world);

    int* operator()(int x, int y);
    int* operator()(double x, double y);
public:
    cv::Point2d origin;
    cv::Point2d resolution;
    cv::Point2i size;
    cv::Point2d state_min;
    cv::Point2d state_max;

    // -2: 占用, -1: 未知, >=0: domain
    std::vector<int> domainMap;
    int numOfDomain;

    std::vector<Layer3ConnectEdge> edges;
    std::map<int/*区域序号*/,std::set<int>/*边的序号*/> domain_edges;
};

class Layer3SearchNode
{
public:
    Layer3SearchNode()
    {
        domainLabel = -1;
        index = std::make_pair(-1, -1);
        via_edge.clear();
        agent_state = cv::Point3d(0, 0, 0);
        dynamic_state.list.clear();
        parent = nullptr;
        g = h = f = 0;
        flag = Layer3GraphNodeFlag::NOT_VISITED;
    }
    ~Layer3SearchNode() = default;

    Layer3SearchNode(Layer3SearchNode& other) = default;
    Layer3SearchNode(const Layer3SearchNode& other) = default;

    Layer3SearchNode& operator=(const Layer3SearchNode& node)
    {
        domainLabel = node.domainLabel;
        index = node.index;
        via_edge = node.via_edge;
        agent_state = node.agent_state;
        dynamic_state = node.dynamic_state;
        path_from_parent = node.path_from_parent;
        parent = node.parent;
        g = node.g;
        h = node.h;
        f = node.f;
        flag = node.flag;
        it = node.it;
        return *this;
    }
public:
    int domainLabel;
    std::pair<int/*通过边序号 */, int/*检查点序号 */> index;
    std::vector<int> via_edge;
    cv::Point3d agent_state;
    uve_message::msg::UveDynamicStatusList dynamic_state;
    layer2::Layer2PlanResult path_from_parent;
    Layer3SearchNode* parent;
    double g, h, f;
    Layer3GraphNodeFlag flag;
    std::multimap<double, Layer3SearchNode*>::iterator it;
};

class Layer3SearchGraph
{
public:
    Layer3SearchGraph(){}
    ~Layer3SearchGraph() = default;

    Layer3SearchGraph(Layer3SearchGraph& other) = delete;
    Layer3SearchGraph(const Layer3SearchGraph& other) = delete;
    Layer3SearchGraph& operator=(const Layer3SearchGraph& other) = delete;

    void fromPixMap(Layer3PixMap& map, std::vector<cv::Point3d>& checkPoints_ref);

    Layer3SearchNode* operator()(int edge_id, int checkPoint_id);
    Layer3SearchNode* operator()(std::pair<int, int> index);
public:
    std::vector<Layer3SearchNode> nodes;
    int edge_num;
    int checkPoint_num;

    // x:长度 ，y:最终旋转角度， z:保留
    std::vector<cv::Point3d> checkPoints;
};

}   // namespace layer3

#endif // LAYER3_DEFINE_HPP