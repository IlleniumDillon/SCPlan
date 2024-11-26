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

class Layer3ConnectNode
{
public:
    Layer3ConnectNode()
    {
        domainLabel = -1;
        g = 0;
        h = 0;
        f = 0;
        parent = -1;
        flag = Layer3GraphNodeFlag::NOT_VISITED;
    }
    ~Layer3ConnectNode() = default;
    Layer3ConnectNode(Layer3ConnectNode& other) = default;
    Layer3ConnectNode(const Layer3ConnectNode& other) = default;
    Layer3ConnectNode& operator=(const Layer3ConnectNode& node)
    {
        domainLabel = node.domainLabel;
        g = node.g;
        h = node.h;
        f = node.f;
        parent = node.parent;
        flag = node.flag;
        return *this;
    }
    bool operator==(const Layer3ConnectNode& node) const
    {
        return domainLabel == node.domainLabel;
    }
public:
    int domainLabel;
    std::vector<int> edgeLabels;
    double g, h, f;
    int parent;
    Layer3GraphNodeFlag flag;
    std::multimap<double, int>::iterator it;

    cv::Point3d agentPose;
    uve_message::msg::UveDynamicStatusList dynamic;
    layer2::Layer2PlanResult path;
};
class Layer3ConnectEdge
{
public:
    Layer3ConnectEdge()
    {
        edgeLabel = -1;
        node1Label = -1;
        node2Label = -1;
        cost = 0;
    }
    Layer3ConnectEdge(int edgeLabel, int node1Label, int node2Label, double cost, std::string name)
        : edgeLabel(edgeLabel), node1Label(node1Label), node2Label(node2Label), cost(cost), name(name) {}
    ~Layer3ConnectEdge() = default;
    Layer3ConnectEdge(Layer3ConnectEdge& other) = default;
    Layer3ConnectEdge(const Layer3ConnectEdge& other) = default;
    Layer3ConnectEdge& operator=(const Layer3ConnectEdge& edge)
    {
        edgeLabel = edge.edgeLabel;
        node1Label = edge.node1Label;
        node2Label = edge.node2Label;
        cost = edge.cost;
        name = edge.name;
        return *this;
    }

    int getOtherNode(int nodeLabel) const
    {
        if (node1Label == nodeLabel)
        {
            return node2Label;
        }
        else if (node2Label == nodeLabel)
        {
            return node1Label;
        }
        return -1;
    }
public:
    int edgeLabel;
    int node1Label;
    int node2Label;
    double cost;
    std::string name;
};
class Layer3ConnectGraph
{
public:
    Layer3ConnectGraph() = default;
    ~Layer3ConnectGraph() = default;
    Layer3ConnectGraph(Layer3ConnectGraph& other) = default;
    Layer3ConnectGraph(const Layer3ConnectGraph& other) = default;
    Layer3ConnectGraph& operator=(const Layer3ConnectGraph& graph)
    {
        nodes = graph.nodes;
        edges = graph.edges;
        return *this;
    }
public:
    std::vector<Layer3ConnectNode> nodes;
    std::vector<Layer3ConnectEdge> edges;
};
class Layer3PixMap
{
public:
    Layer3PixMap() = default;
    ~Layer3PixMap() = default;
    Layer3PixMap(Layer3PixMap& other) = delete;
    Layer3PixMap(const Layer3PixMap& other) = delete;
    Layer3PixMap& operator=(const Layer3PixMap& other) = delete;

    void fromLayer1Graph(layer1::Layer1GridGraph& graph);

    int operator()(int x, int y);
    int operator()(double x, double y);

    void copyFrom(Layer3PixMap& other);

public:
    cv::Point2d origin;
    cv::Point2d resolution;
    cv::Point2i size;
    cv::Point2d state_min;
    cv::Point2d state_max;
    // -1: 未知， 0: 占用， >0: 连通域号
    std::vector<int> data;
    int maxDomainLabel = 0;
};

}   // namespace layer3

#endif // LAYER3_DEFINE_HPP