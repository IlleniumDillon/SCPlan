#ifndef LAYER3_PLAN_HPP
#define LAYER3_PLAN_HPP

#include "layer3/layer3_define.hpp"

namespace layer3
{

class Layer3PlanResult
{
public:
    Layer3PlanResult() = default;
    ~Layer3PlanResult() = default;
public:
    bool success = false;
    double planTime = std::numeric_limits<double>::max();
    double cost = std::numeric_limits<double>::max();
    std::vector<layer2::Layer2PlanResult> path;
};

class Layer3Plan
{
public:
    Layer3Plan() = default;
    ~Layer3Plan() = default;

    void bindGraph(Layer3SearchGraph* graph);
    void bindPixMap(Layer3PixMap* map);
    void bindPlanner(layer2::Layer2Plan* planner);

    void getNeighbors(Layer3SearchNode *node,
                                std::vector<Layer3SearchNode *> &neighbors,
                                std::vector<double> &costs,
                                std::vector<int> &neighbor_domain,
                                std::vector<cv::Point3f> &neighbor_agent_state,
                                std::vector<uve_message::msg::UveDynamicStatusList> &neighbor_dynamic_state,
                                std::vector<layer2::Layer2PlanResult> &neighbor_path_from_parent,
                                std::vector<int> &neighbor_via_edge,
                                std::vector<std::pair<int, int>> &neighbor_index);

    Layer3PlanResult search(cv::Point3d start, cv::Point3d goal);

public:
    Layer3SearchGraph* graph;
    Layer3PixMap* map;
    layer2::Layer2Plan* planner;

    std::multimap<double, Layer3SearchNode*> open_set;

    int goalDomain;
    cv::Point3d goal;
};


}  // namespace layer3

#endif // LAYER3_PLAN_HPP