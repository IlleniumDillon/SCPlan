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
    Layer3Plan();
    ~Layer3Plan() = default;

    void setMaxThread(int num);
    void setInitGraph(layer1::Layer1GridGraph& freeGraph, layer1::Layer1GridGraph& carryGraph);
    void setInitGraph(std::shared_ptr<layer1::Layer1GridGraph> freeGraph, std::shared_ptr<layer1::Layer1GridGraph> carryGraph);
    void setWorldDSCP(uvs_message::srv::UvQueryWorld::Response& w);
    void setFreeExecuteSpace(double max_v, double max_w, int step_v, int step_w, double dt);
    void setCarryExecuteSpace(double max_v, double max_w, int step_v, int step_w, double dt);
    void updateGraph(uve_message::msg::UveDynamicStatusList& nstate);
    Layer3PlanResult search(cv::Point3d start, cv::Point3d goal);
    void getNeighbors(int curDomainLabel,
                      std::vector<int>& neighbors,
                      std::vector<cv::Point3d>& neighbor_agent_pose,
                      std::vector<uve_message::msg::UveDynamicStatusList>& neighbor_dynamic,
                      std::vector<layer2::Layer2PlanResult>& neighbor_path);
public:
    uvs_message::srv::UvQueryWorld::Response world;

    layer1::Layer1GridGraph freeGraph;
    layer1::Layer1GridGraph carryGraph;
    layer3::Layer3ConnectGraph connectGraph;
    layer3::Layer3PixMap pixMap;

    uve_message::msg::UveDynamicStatusList dynamic_state;

    layer1::Layer1Plan layer1Plan;
    layer2::Layer2Plan layer2Plan;

    int startDomainLabel;
    int goalDomainLabel;
    cv::Point3d startAgentPose;
    cv::Point3d goalAgentPose;

    std::vector <cv::Point3d> cargoCheckPose;
};

}  // namespace layer3

#endif // LAYER3_PLAN_HPP