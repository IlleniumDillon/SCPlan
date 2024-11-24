#ifndef LAYER1_PLAN_HPP
#define LAYER1_PLAN_HPP

#include "layer1_define.hpp"

namespace layer1
{
class Layer1PlanResult
{
public:
    Layer1PlanResult() = default;
    ~Layer1PlanResult() = default;
public:
    bool success = false;
    int iterations = 0;
    double planTime = std::numeric_limits<double>::max();
    double cost = std::numeric_limits<double>::max();
    std::vector<cv::Point3d> path;
    std::vector<cv::Point2d> vw;
};
class Layer1Plan
{
public:
    Layer1Plan() = default;
    ~Layer1Plan() = default;

    void setExecuteSpace(double max_v, double max_w, int step_v, int step_w, double dt);
    void bindGraph(Layer1GridGraph* graph);
    void reset();
    Layer1GraphNodeCost heuristic(Layer1GraphNode* node, Layer1GraphNode* goal);
    void getNeighbors(Layer1GraphNode* node, 
        std::vector<Layer1GraphNode*>& neighbors,
        std::vector<Layer1GraphNodeCost>& costs,
        std::vector<cv::Point3f>& neighbor_state,
        std::vector<cv::Point2d>& vws);
    bool endCondition(Layer1GraphNode* node, Layer1GraphNode* goal);
    Layer1PlanResult search(Layer1GraphNode* start, Layer1GraphNode* goal);
    Layer1PlanResult search(cv::Point3d start, cv::Point3d goal);

public:
    std::multimap<Layer1GraphNodeCost, Layer1GraphNode*> open_set;
    std::vector<Layer1GraphNode*> close_set;
    Layer1GridGraph* graph;
    Layer1PlanResult result;

    std::vector<cv::Point2d> vw_space;
    // 0: 圆弧割线长度, 1: 割线与切线夹角， 2: 转角
    std::vector<cv::Point3d> neighbor_space;
    std::vector<double> neighbor_cost;
};
} // namespace layer1

#endif // LAYER1_PLAN_HPP