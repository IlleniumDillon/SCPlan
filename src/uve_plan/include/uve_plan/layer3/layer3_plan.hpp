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
    void setMaxThread(int num) {    max_thread = num;    }

    void getNeighbors(Layer3SearchNode *node,
                                std::vector<Layer3SearchNode *> &neighbors,
                                std::vector<double> &costs,
                                std::vector<int> &neighbor_domain,
                                std::vector<cv::Point3f> &neighbor_agent_state,
                                std::vector<uve_message::msg::UveDynamicStatusList> &neighbor_dynamic_state,
                                std::vector<layer2::Layer2PlanResult> &neighbor_path_from_parent,
                                std::vector<int> &neighbor_via_edge,
                                std::vector<std::pair<int, int>> &neighbor_index);
    struct threadout
    {
        Layer3SearchNode *neighbor;
        double cost;
        int domain;
        cv::Point3f agent_state;
        uve_message::msg::UveDynamicStatusList dynamic_state;
        layer2::Layer2PlanResult path_from_parent;
        int via_edge;
        std::pair<int, int> index;
    };
    struct threadin
    {
        cv::Point3d Astart;
        std::string Cname;
        cv::Point3d Cgoal;
        cv::Point3d Agoal;
        uve_message::msg::UveDynamicStatusList dynamic_state;
        Layer3SearchNode *node;
        int edge_id;
        int checkPoint_id;
    };
    threadout searchThread(int id, threadin& in);

    Layer3PlanResult search(cv::Point3d start, cv::Point3d goal);
    double heuristic(Layer3SearchNode* node, cv::Point3d goal);

public:
    Layer3SearchGraph* graph;
    Layer3PixMap* map;
    layer2::Layer2Plan* planner;

    std::multimap<double, Layer3SearchNode*> open_set;

    int goalDomain;
    cv::Point3d goal;

    int max_thread = 1;
};


}  // namespace layer3

#endif // LAYER3_PLAN_HPP