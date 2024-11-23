#include "layer2_plan.hpp"

using namespace layer2;

void Layer2Plan::setMaxThread(int num)
{
    this->max_thread = num;
    plans = std::vector<layer1::Layer1Plan>(num);
    freeGraphs = std::vector<layer1::Layer1GridGraph>(num);
    carryGraphs = std::vector<layer1::Layer1GridGraph>(num);
}

void Layer2Plan::setInitGraph(layer1::Layer1GridGraph &graph)
{
    this->initGraph.copyFrom(graph);
}

void Layer2Plan::setFreeExecuteSpace(double max_v, double max_w, int step_v, int step_w, double dt)
{
    freeConfig.clear();
    freeConfig = {max_v, max_w, (double)step_v, (double)step_w, dt};
}

void Layer2Plan::setCarryExecuteSpace(double max_v, double max_w, int step_v, int step_w, double dt)
{
    carryConfig.clear();
    carryConfig = {max_v, max_w, (double)step_v, (double)step_w, dt};
}

Layer2PlanResult Layer2Plan::search(cv::Point3d Astate, std::string Cname, cv::Point3d Cgoal, cv::Point3d Agoal)
{
    // 退化情况，不携带物体
    if (Cname == "")
    {
        layer1::Layer1Plan plan;
        plan.setExecuteSpace(freeConfig[0], freeConfig[1], freeConfig[2], freeConfig[3], freeConfig[4]);
        plan.bindGraph(initGraph);
        auto ret = plan.search(Astate, Agoal);
        Layer2PlanResult result;
        if (ret.success)
        {
            result.success = true;
            result.cost = ret.cost;
            result.path_m = ret.path;
            result.vw_m = ret.vw;
        }
        return result;
    }

    // 正常情况，携带物体
    auto targetCargoIt = initGraph.dynamic_map.find(Cname);
    if (targetCargoIt == initGraph.dynamic_map.end())
    {
        return Layer2PlanResult();
    }
    
}

Layer2PlanResult Layer2Plan::searchThread(cv::Point3d Astart, cv::Point3d CAstart, cv::Point3d CAgoal, cv::Point3d Agoal)
{
    return Layer2PlanResult();
}
