#ifndef LAYER2_PLAN_HPP
#define LAYER2_PLAN_HPP

#include "layer2_define.hpp"

#include <thread>
#include <future>

namespace layer2
{
class Layer2PlanResult
{
public:
    Layer2PlanResult() = default;
    ~Layer2PlanResult() = default;
public:
    bool success = false;
    double planTime = std::numeric_limits<double>::max();
    double cost = std::numeric_limits<double>::max();
    std::vector<cv::Point3d> path_m;
    std::vector<cv::Point3d> path_c;
    std::vector<cv::Point3d> path_a;
    std::vector<cv::Point2d> vw_m;
    std::vector<cv::Point2d> vw_c;
    std::vector<cv::Point2d> vw_a;
};

class Layer2Plan
{
public:
    Layer2Plan() = default;
    ~Layer2Plan() = default;

    void setMaxThread(int num);
    void setInitGraph(layer1::Layer1GridGraph& freeGraph, layer1::Layer1GridGraph& carryGraph);
    void setInitGraph(std::shared_ptr<layer1::Layer1GridGraph> freeGraph, std::shared_ptr<layer1::Layer1GridGraph> carryGraph);
    void setWorldDSCP(uvs_message::srv::UvQueryWorld::Response& w);
    void setFreeExecuteSpace(double max_v, double max_w, int step_v, int step_w, double dt);
    void setCarryExecuteSpace(double max_v, double max_w, int step_v, int step_w, double dt);
    void updateGraph(uve_message::msg::UveDynamicStatusList& nstate);
    Layer2PlanResult search(cv::Point3d Astate, std::string Cname, cv::Point3d Cgoal, cv::Point3d Agoal);
public:
    int max_thread = 1;
    // layer1::Layer1GridGraph freeGraph;
    // layer1::Layer1GridGraph carryGraph;
    uvs_message::srv::UvQueryWorld::Response world;
    std::vector<double> freeConfig;
    std::vector<double> carryConfig;

    uve_message::msg::UveDynamicStatusList dynamic_state;

    std::vector<layer1::Layer1Plan> plans;
    std::vector<layer1::Layer1GridGraph> freeGraphs;
    std::vector<layer1::Layer1GridGraph> carryGraphs;
    // std::vector<std::future<Layer2PlanResult>> futures;
    std::multimap<double, Layer2PlanResult> results;

    std::string cur_Cname;
    cv::Point3d cur_Cgoal;

private:
    Layer2PlanResult searchThread(int id, cv::Point3d Astart, cv::Point3d CAstart, cv::Point3d CAgoal, cv::Point3d Agoal);
};

} // namespace layer2

#endif // LAYER2_PLAN_HPP